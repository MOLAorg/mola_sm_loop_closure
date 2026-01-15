/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mola_georeferencing/simplemap_georeference.h>
#include <mola_gtsam_factors/FactorGnssEnu.h>
#include <mola_sm_loop_closure/FrameToFrameLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/system/filesystem.h>

using namespace mola;

IMPLEMENTS_SERIALIZABLE(FrameToFrameLoopClosure, LoopClosureInterface, mola)

namespace
{
const bool PRINT_LC_SCORES = mrpt::get_env<bool>("PRINT_LC_SCORES", false);
const bool SAVE_ICP_LOGS   = mrpt::get_env<bool>("SAVE_ICP_LOGS", false);

bool frame_has_mapping_observations(const mrpt::obs::CSensoryFrame& sf)
{
    if (sf.empty())
    {
        return false;
    }

    if (sf.getObservationByClass<mrpt::obs::CObservationPointCloud>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservation2DRangeScan>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservationVelodyneScan>())
    {
        return true;
    }

    return false;
}

}  // namespace

FrameToFrameLoopClosure::FrameToFrameLoopClosure()
{
    mrpt::system::COutputLogger::setLoggerName("FrameToFrameLoopClosure");
    threads_.name("f2f_icp_threads");
}

void FrameToFrameLoopClosure::initialize(const mrpt::containers::yaml& c)
{
    MRPT_TRY_START

    const auto cfg = c["params"];

    // Load parameters
    YAML_LOAD_OPT(params_, use_gnss, bool);
    YAML_LOAD_OPT(params_, gnss_minimum_uncertainty_xyz, double);
    YAML_LOAD_OPT(params_, gnss_add_horizontality, bool);
    YAML_LOAD_OPT(params_, gnss_horizontality_sigma_z, double);

    YAML_LOAD_OPT(params_, min_distance_between_frames, double);
    YAML_LOAD_OPT(params_, max_distance_for_lc_candidate, double);
    YAML_LOAD_OPT(params_, max_lc_candidates, size_t);
    YAML_LOAD_OPT(params_, min_frames_between_lc, size_t);
    YAML_LOAD_OPT(params_, max_lc_optimization_rounds, size_t);

    if (params_.min_frames_between_lc == 0)
    {
        MRPT_LOG_WARN("min_frames_between_lc=0 is invalid; clamping to 1.");
        params_.min_frames_between_lc = 1;
    }

    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, icp_edge_robust_param, double);
    YAML_LOAD_OPT(params_, icp_edge_additional_noise_xyz, double);
    YAML_LOAD_OPT(params_, icp_edge_additional_noise_ang, double);
    YAML_LOAD_OPT(params_, threshold_sigma_initial, std::string);
    YAML_LOAD_OPT(params_, threshold_sigma_final, std::string);

    YAML_LOAD_OPT(params_, input_odometry_noise_xyz, double);
    YAML_LOAD_OPT(params_, input_odometry_noise_ang, double);
    YAML_LOAD_OPT(params_, input_edges_uncertainty_multiplier, double);

    YAML_LOAD_OPT(params_, largest_delta_for_reconsider, double);
    YAML_LOAD_OPT(params_, max_sensor_range, double);

    YAML_LOAD_OPT(params_, profiler_enabled, bool);
    YAML_LOAD_OPT(params_, save_trajectory_files, bool);
    YAML_LOAD_OPT(params_, debug_files_prefix, std::string);

    profiler_.enable(params_.profiler_enabled);

    // Initialize ICP pipelines for each thread
    ENSURE_YAML_ENTRY_EXISTS(c, "icp_settings");

    for (auto& pts : state_.perThreadState_)
    {
        const auto [icp, icpParams] = mp2p_icp::icp_pipeline_from_yaml(c["icp_settings"]);
        pts.icp                     = icp;
        params_.icp_parameters      = icpParams;

        pts.icp->attachToParameterSource(pts.parameter_source);

        // Observation generators
        if (c.has("observations_generator") && !c["observations_generator"].isNullNode())
        {
            pts.obs_generators =
                mp2p_icp_filters::generators_from_yaml(c["observations_generator"]);
        }
        else
        {
            auto defaultGen = mp2p_icp_filters::Generator::Create();
            defaultGen->initialize({});
            pts.obs_generators.push_back(defaultGen);
        }
        mp2p_icp::AttachToParameterSource(pts.obs_generators, pts.parameter_source);

        // Observation filters
        if (c.has("observations_filter"))
        {
            pts.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(c["observations_filter"]);
            mp2p_icp::AttachToParameterSource(pts.pc_filter, pts.parameter_source);
        }
    }

    state_.initialized = true;

    MRPT_TRY_END
}

void FrameToFrameLoopClosure::process(mrpt::maps::CSimpleMap& sm)
{
    using namespace std::string_literals;

    ASSERT_(state_.initialized);
    state_.sm = &sm;

    MRPT_LOG_INFO_STREAM("Processing simplemap with " << sm.size() << " frames");

    // Build initial graph with odometry edges
    build_initial_graph();

    // Add GNSS factors if available
    if (params_.use_gnss)
    {
        add_gnss_factors();
    }

    if (params_.save_trajectory_files)
    {
        save_trajectory_as_tum(params_.debug_files_prefix + "initial.tum"s);
    }

    // Initial optimization with GNSS
    if (params_.use_gnss)
    {
        MRPT_LOG_INFO("Running initial GNSS optimization...");
        optimize_graph();

        if (params_.save_trajectory_files)
        {
            save_trajectory_as_tum(params_.debug_files_prefix + "after_gnss.tum"s);
        }
    }

    // Loop closure detection and optimization
    size_t                                      accepted_lcs = 0;
    std::set<std::pair<frame_id_t, frame_id_t>> alreadyChecked;

    for (size_t lcRound = 0; lcRound < params_.max_lc_optimization_rounds; lcRound++)
    {
        size_t checkedCount   = 0;
        bool   anyGraphChange = false;

        auto candidates = find_loop_candidates(alreadyChecked);

        MRPT_LOG_INFO_STREAM("Found " << candidates.size() << " loop closure candidates");

        const auto frameGroup = static_cast<double>(params_.min_frames_between_lc);

        for (const auto& lc : candidates)
        {
            // Decimate the frame IDs so we are effectively counting "blocks" of frames for what
            // concerns already-checked:
            const auto frameGroup_i = mrpt::round(static_cast<double>(lc.frame_i) / frameGroup);
            const auto frameGroup_j = mrpt::round(static_cast<double>(lc.frame_j) / frameGroup);

            const auto IDs = std::make_pair(
                std::min<frame_id_t>(frameGroup_i, frameGroup_j),
                std::max<frame_id_t>(frameGroup_i, frameGroup_j));

            if (alreadyChecked.count(IDs) != 0)
            {
                continue;
            }

            alreadyChecked.insert(IDs);
            checkedCount++;

            MRPT_LOG_INFO_STREAM(
                "Checking LC candidate: " << lc.frame_i << " <-> " << lc.frame_j
                                          << " distance=" << lc.distance << " score=" << lc.score);

            const bool accepted = process_loop_candidate(lc);
            if (accepted)
            {
                anyGraphChange = true;
                accepted_lcs++;
            }
        }

        if (checkedCount == 0)
        {
            break;  // No new candidates
        }

        if (anyGraphChange)
        {
            const double largestDelta = optimize_graph();

            if (largestDelta > params_.largest_delta_for_reconsider)
            {
                MRPT_LOG_INFO_STREAM(
                    "Large pose change detected (" << largestDelta
                                                   << "m), reconsidering all candidates");
                alreadyChecked.clear();
            }
        }
    }

    MRPT_LOG_INFO_STREAM("Total accepted loop closures: " << accepted_lcs);

    if (params_.save_trajectory_files)
    {
        save_trajectory_as_tum(params_.debug_files_prefix + "final.tum"s);
    }

    // Update simplemap with optimized poses
    mrpt::maps::CSimpleMap outSM;
    for (size_t id = 0; id < sm.size(); id++)
    {
        auto& [oldPose, sf, twist] = sm.get(id);

        const auto newPose = mrpt::poses::CPose3DPDFGaussian::Create();
        newPose->mean      = state_.get_pose(id);
        newPose->cov.setIdentity();

        outSM.insert(newPose, sf, twist);
    }

    sm = outSM;  // TODO: Make CSimpleMap move constructible
}

void FrameToFrameLoopClosure::build_initial_graph()
{
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "build_initial_graph");

    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    // Add all frame poses to values
    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto pose_i = frame_pose_in_simplemap(i);
        state_.graphValues.insert(X(i), mrpt::gtsam_wrappers::toPose3(pose_i));
    }

    // Add prior on first frame: very weak, so GNSS can override it as needed.
    const auto pose0      = frame_pose_in_simplemap(0);
    auto       priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e+3);

    state_.graphFG.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(0), mrpt::gtsam_wrappers::toPose3(pose0), priorNoise);

    // Add odometry edges between consecutive frames
    for (size_t i = 1; i < sm.size(); i++)
    {
        const auto pose_i   = frame_pose_in_simplemap(i);
        const auto pose_im1 = frame_pose_in_simplemap(i - 1);

        const auto relPose   = pose_i - pose_im1;
        const auto deltaPose = mrpt::gtsam_wrappers::toPose3(relPose);

        gtsam::Vector6 sigmas;
        sigmas << mrpt::DEG2RAD(params_.input_odometry_noise_ang),
            mrpt::DEG2RAD(params_.input_odometry_noise_ang),
            mrpt::DEG2RAD(params_.input_odometry_noise_ang), params_.input_odometry_noise_xyz,
            params_.input_odometry_noise_xyz, params_.input_odometry_noise_xyz;

        sigmas *= params_.input_edges_uncertainty_multiplier;

        auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

        auto factor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);

        state_.graphFG += factor;
    }

    MRPT_LOG_INFO_STREAM("Built initial graph with " << sm.size() << " frames");
}

void FrameToFrameLoopClosure::add_gnss_factors()
{
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "add_gnss_factors");

    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    // Extract GNSS frames
    AddGNSSFactorParams gpsParams;
    gpsParams.minimumUncertaintyXYZ       = params_.gnss_minimum_uncertainty_xyz;
    gpsParams.addHorizontalityConstraints = params_.gnss_add_horizontality;
    gpsParams.horizontalitySigmaZ         = params_.gnss_horizontality_sigma_z;

    const auto gnssFrames = extract_gnss_frames_from_sm(sm, state_.globalGeoRef);

    if (gnssFrames.frames.empty())
    {
        MRPT_LOG_WARN("No valid GNSS observations found");
        return;
    }

    if (!state_.globalGeoRef.has_value())
    {
        state_.globalGeoRef = gnssFrames.refCoord;
    }

    MRPT_LOG_INFO_STREAM("Adding " << gnssFrames.frames.size() << " GNSS factors");

    // Add GNSS factors for each frame
    for (const auto& gf : gnssFrames.frames)
    {
        // Find which frame this corresponds to
        frame_id_t frameId = 0;
        bool       found   = false;

        for (size_t i = 0; i < sm.size(); i++)
        {
            const auto& [pose, sf, twist] = sm.get(i);

            for (const auto& obs : *sf)
            {
                if (obs.get() == gf.obs.get())
                {
                    frameId = i;
                    found   = true;
                    break;
                }
            }
            if (found)
            {
                break;
            }
        }

        if (!found)
        {
            continue;
        }

        auto noiseOrg =
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(gf.sigma_E, gf.sigma_N, gf.sigma_U)
                                                    .array()
                                                    .max(params_.gnss_minimum_uncertainty_xyz));

        auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.5), noiseOrg);

        const auto observedENU = mrpt::gtsam_wrappers::toPoint3(gf.enu);
        const auto sensorPointOnVeh =
            mrpt::gtsam_wrappers::toPoint3(gf.obs->sensorPose.translation());

        state_.graphFG.emplace_shared<mola::factors::FactorGnssEnu>(
            X(frameId), sensorPointOnVeh, observedENU, robustNoise);
    }
}

std::vector<FrameToFrameLoopClosure::LoopCandidate> FrameToFrameLoopClosure::find_loop_candidates(
    const std::set<std::pair<frame_id_t, frame_id_t>>& alreadyChecked) const
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "find_loop_candidates");

    std::vector<LoopCandidate> candidates;

    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    const auto frameGroup = static_cast<double>(params_.min_frames_between_lc);

    // Compare each frame against potential loop closure frames
    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto pose_i = state_.get_pose(i);

        // Look for candidates that are:
        // 1. Far enough in frame index
        // 2. Close enough in space
        // 3. Not already checked
        for (size_t j = i + params_.min_frames_between_lc; j < sm.size(); j++)
        {
            // Decimate the frame IDs so we are effectively counting "blocks" of frames for what
            // concerns already-checked:
            const auto frameGroup_i = mrpt::round(static_cast<double>(i) / frameGroup);
            const auto frameGroup_j = mrpt::round(static_cast<double>(j) / frameGroup);

            const auto IDs = std::make_pair(
                std::min<frame_id_t>(frameGroup_i, frameGroup_j),
                std::max<frame_id_t>(frameGroup_i, frameGroup_j));

            if (alreadyChecked.count(IDs) != 0)
            {
                continue;
            }

            const auto   pose_j   = state_.get_pose(j);
            const double distance = (pose_i.translation() - pose_j.translation()).norm();

            // Check distance criteria
            if (distance < params_.min_distance_between_frames ||
                distance > params_.max_distance_for_lc_candidate)
            {
                continue;
            }

            // Check both frames have valid observations
            const auto& [_, sf_i, __]     = sm.get(i);  // NOLINT(bugprone-reserved-identifier)
            const auto& [___, sf_j, ____] = sm.get(j);  // NOLINT(bugprone-reserved-identifier)

            if (!frame_has_mapping_observations(*sf_i) || !frame_has_mapping_observations(*sf_j))
            {
                continue;
            }

            LoopCandidate lc;
            lc.frame_i  = i;
            lc.frame_j  = j;
            lc.distance = distance;
            lc.score    = 1.0 / (1.0 + distance);  // Closer = better score

            candidates.push_back(lc);

            if (PRINT_LC_SCORES)
            {
                MRPT_LOG_DEBUG_STREAM(
                    "Candidate: " << i << " <-> " << j << " dist=" << distance
                                  << " score=" << lc.score);
            }
        }
    }

    // Sort by score (best first)
    std::sort(
        candidates.begin(), candidates.end(),
        [](const LoopCandidate& a, const LoopCandidate& b) { return a.score > b.score; });

    // Limit to max candidates
    if (candidates.size() > params_.max_lc_candidates)
    {
        candidates.resize(params_.max_lc_candidates);
    }

    return candidates;
}

bool FrameToFrameLoopClosure::process_loop_candidate(const LoopCandidate& lc)
{
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "process_loop_candidate");

    const size_t threadIdx = 0;  // Use first thread for now

    // Generate point clouds for both frames
    auto pc_i = generate_frame_pointcloud(lc.frame_i, threadIdx);
    auto pc_j = generate_frame_pointcloud(lc.frame_j, threadIdx);

    if (!pc_i || !pc_j)
    {
        MRPT_LOG_WARN_STREAM(
            "Failed to generate point clouds for LC " << lc.frame_i << " <-> " << lc.frame_j);
        return false;
    }

    // Initial guess from current graph
    const auto pose_i    = state_.get_pose(lc.frame_i);
    const auto pose_j    = state_.get_pose(lc.frame_j);
    const auto initGuess = (pose_j - pose_i).asTPose();

    // Run ICP
    auto& pts = state_.perThreadState_.at(threadIdx);

    update_dynamic_variables(lc.frame_j, threadIdx);

    mp2p_icp::Results icp_result;
    pts.icp->align(*pc_j, *pc_i, initGuess, params_.icp_parameters, icp_result);

    MRPT_LOG_INFO_STREAM(
        "ICP " << lc.frame_i << " <-> " << lc.frame_j << ": quality="
               << (100.0 * icp_result.quality) << "% iters=" << icp_result.nIterations << " delta="
               << (icp_result.optimal_tf.getMeanVal().asTPose() - initGuess).asString());

    if (icp_result.quality < params_.min_icp_goodness)
    {
        return false;
    }

    // Add ICP edge to graph
    const auto deltaPose = mrpt::gtsam_wrappers::toPose3(icp_result.optimal_tf.mean);

    gtsam::Vector6 sigmas;
    const auto     covDiag = icp_result.optimal_tf.cov.asEigen().diagonal().array().sqrt();

    sigmas << covDiag[5] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[4] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[3] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[0] + params_.icp_edge_additional_noise_xyz,
        covDiag[1] + params_.icp_edge_additional_noise_xyz,
        covDiag[2] + params_.icp_edge_additional_noise_xyz;

    auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    // Robust kernel:
    auto robustNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::GemanMcClure::Create(params_.icp_edge_robust_param),
        edgeNoise);

    state_.graphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(lc.frame_i), X(lc.frame_j), deltaPose, robustNoise);

    return true;
}

// TODO: Move to mp2p_icp module
namespace
{
void processLocalVelocityBuffer(
    const mrpt::obs::CObservation::Ptr& obs, mp2p_icp::ParameterSource& ps)
{
    auto obsComment = std::dynamic_pointer_cast<mrpt::obs::CObservationComment>(obs);
    if (!obsComment)
    {
        return;
    }

    const auto commentYaml = [&]()
    {
        try
        {
            return mrpt::containers::yaml::FromText(obsComment->text);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing YAML in comment: " << e.what() << "\n";
            return mrpt::containers::yaml();
        }
    }();

    if (!commentYaml.isMap() || !commentYaml.has("local_velocity_buffer"))
    {
        return;
    }

    const auto lvb = commentYaml["local_velocity_buffer"];
    if (!lvb.isMap())
    {
        std::cerr << "Error: 'local_velocity_buffer' field is not a map!\n";
        return;
    }

    try
    {
        ps.localVelocityBuffer.fromYAML(lvb);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error parsing 'local_velocity_buffer': " << e.what() << "\n";
        return;
    }
};
}  // namespace

mp2p_icp::metric_map_t::Ptr FrameToFrameLoopClosure::generate_frame_pointcloud(
    frame_id_t frameId, size_t threadIdx)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "generate_frame_pointcloud");

    ASSERT_(state_.sm);
    const auto& [pose, sf, twist] = state_.sm->get(frameId);

    if (!frame_has_mapping_observations(*sf))
    {
        return nullptr;
    }

    auto& pts         = state_.perThreadState_.at(threadIdx);
    auto  observation = mp2p_icp::metric_map_t::Create();

    // First, search for velocity buffer data:
    for (const auto& obs : *sf)
    {
        ASSERT_(obs);
        processLocalVelocityBuffer(obs, pts.parameter_source);
    }

    update_dynamic_variables(frameId, threadIdx);

    // Next, do the actual sensor data processing:
    // Generate point cloud from observations
    for (const auto& obs : *sf)
    {
        mp2p_icp_filters::apply_generators(pts.obs_generators, *obs, *observation);
    }

    // Apply filters
    mp2p_icp_filters::apply_filter_pipeline(pts.pc_filter, *observation, profiler_);

    return observation;
}

double FrameToFrameLoopClosure::optimize_graph()
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "optimize_graph");

    auto lmParams = gtsam::LevenbergMarquardtParams::CeresDefaults();

    ASSERT_(!state_.graphFG.empty());

    const auto N_1 = 1.0 / static_cast<double>(state_.graphFG.size());

    const double errInit1  = state_.graphFG.error(state_.graphValues);
    const double rmseInit1 = std::sqrt(errInit1 * N_1);

    gtsam::LevenbergMarquardtOptimizer lm1(state_.graphFG, state_.graphValues, lmParams);
    const auto                         optimalValues = lm1.optimize();

    const double errEnd1  = state_.graphFG.error(optimalValues);
    const double rmseEnd1 = std::sqrt(errEnd1 * N_1);

    // Compute largest pose change
    double largestDelta = 0.0;
    using gtsam::symbol_shorthand::X;

    for (size_t i = 0; i < state_.sm->size(); i++)
    {
        const auto newPose = mrpt::gtsam_wrappers::toTPose3D(optimalValues.at<gtsam::Pose3>(X(i)));
        const auto oldPose =
            mrpt::gtsam_wrappers::toTPose3D(state_.graphValues.at<gtsam::Pose3>(X(i)));

        const double delta = mrpt::poses::CPose3D(oldPose - newPose).translation().norm();
        mrpt::keep_max(largestDelta, delta);
    }

    // Save new optimal values
    state_.graphValues = optimalValues;

    auto bckCol =
        mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO);
    mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO) =
        mrpt::system::ConsoleForegroundColor::BRIGHT_GREEN;

    MRPT_LOG_INFO_STREAM(
        "Graph optimized: " << lm1.iterations() << " iters, RMSE " << rmseInit1 << " -> "
                            << rmseEnd1 << ", largest delta: " << largestDelta << " m");

    mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO) = bckCol;

    return largestDelta;
}

mrpt::poses::CPose3D FrameToFrameLoopClosure::frame_pose_in_simplemap(frame_id_t frameId) const
{
    ASSERT_(state_.sm);
    const auto& [pose, sf, twist] = state_.sm->get(frameId);
    ASSERT_(pose);
    return pose->getMeanVal();
}

mrpt::poses::CPose3D FrameToFrameLoopClosure::State::get_pose(frame_id_t id) const
{
    using gtsam::symbol_shorthand::X;
    return mrpt::poses::CPose3D(
        mrpt::gtsam_wrappers::toTPose3D(graphValues.at<gtsam::Pose3>(X(id))));
}

void FrameToFrameLoopClosure::update_dynamic_variables(frame_id_t frameId, size_t threadIdx)
{
    auto& pts = state_.perThreadState_.at(threadIdx);
    auto& ps  = pts.parameter_source;

    const auto& [pose, sf, twist] = state_.sm->get(frameId);

    // Set twist for deskewing
    mrpt::math::TTwist3D twistForIcp = {0, 0, 0, 0, 0, 0};
    if (twist)
    {
        twistForIcp = *twist;
    }

    ps.updateVariable("vx", twistForIcp.vx);
    ps.updateVariable("vy", twistForIcp.vy);
    ps.updateVariable("vz", twistForIcp.vz);
    ps.updateVariable("wx", twistForIcp.wx);
    ps.updateVariable("wy", twistForIcp.wy);
    ps.updateVariable("wz", twistForIcp.wz);

    if (!pts.expr_threshold_sigma_final.is_compiled())
    {
        pts.expr_threshold_sigma_final.compile(
            params_.threshold_sigma_final, {}, "expr_threshold_sigma_final");

        pts.expr_threshold_sigma_initial.compile(
            params_.threshold_sigma_initial, {}, "expr_threshold_sigma_initial");
    }

    ps.updateVariable("SIGMA_INIT", pts.expr_threshold_sigma_initial.eval());
    ps.updateVariable("SIGMA_FINAL", pts.expr_threshold_sigma_final.eval());
    ps.updateVariable("ESTIMATED_SENSOR_MAX_RANGE", params_.max_sensor_range);

    // This will be overwritten by the actual ICP loop later on,
    // but we need to define all variables before building a local map:
    ps.updateVariable("ICP_ITERATION", 0);

    ps.realize();
}

void FrameToFrameLoopClosure::save_trajectory_as_tum(const std::string& filename) const
{
    ASSERT_(state_.sm);

    mrpt::poses::CPose3DInterpolator path;

    for (size_t id = 0; id < state_.sm->size(); id++)
    {
        const auto& [oldPose, sf, twist] = state_.sm->get(id);
        const auto newPose               = state_.get_pose(id);

        if (sf->empty())
        {
            MRPT_LOG_WARN_STREAM("Frame " << id << " has no observations, skipping in trajectory");
            continue;
        }
        const auto t = sf->getObservationByIndex(0)->timestamp;

        path.insert(t, newPose);
    }

    path.saveToTextFile_TUM(filename);
    MRPT_LOG_INFO_STREAM("Saved trajectory to: " << filename);
}