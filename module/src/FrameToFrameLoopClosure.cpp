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
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mola_georeferencing/simplemap_georeference.h>
#include <mola_gtsam_factors/FactorGnssEnu.h>
#include <mola_gtsam_factors/MeasuredGravityFactor.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>
#include <mola_sm_loop_closure/FrameToFrameLoopClosure.h>
#include <mola_sm_loop_closure/common/debug_flags.h>
#include <mola_sm_loop_closure/common/gnc_optimizer.h>
#include <mola_sm_loop_closure/common/gnss_factor_helpers.h>
#include <mola_sm_loop_closure/common/icp_pipeline_setup.h>
#include <mola_sm_loop_closure/common/obs_helpers.h>
#include <mola_sm_loop_closure/common/planarity_factors.h>
#include <mola_sm_loop_closure/common/tum_writer.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/update_velocity_buffer_from_obs.h>
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/Viewport.h>
#include <mrpt/viz/opengl_fonts.h>

#ifdef MOLA_HAS_KISS_MATCHER
#include <kiss_matcher/KISSMatcher.hpp>
#endif

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <future>
#include <mutex>

#include "DeterministicScope.h"

using namespace mola;

IMPLEMENTS_SERIALIZABLE(FrameToFrameLoopClosure, LoopClosureInterface, mola)

namespace
{
// Convenience shortcuts to the shared debug-flag singleton.
#define PRINT_LC_SCORES (mola::lc_common::DebugFlags::instance().print_lc_scores)
#define SAVE_ICP_LOGS (mola::lc_common::DebugFlags::instance().save_icp_logs)

using mola::lc_common::frame_has_mapping_observations;

/**
 * Compute score using original proximity-only strategy
 */
double score_proximity_only(double distance) { return 1.0 / (1.0 + distance); }

/**
 * Compute score for distance-stratified strategy
 * Combines proximity with frame separation
 */
double score_stratified(
    double distance, double minDist, double maxDist, size_t frameI, size_t frameJ,
    size_t totalFrames)
{
    const double distRange = maxDist - minDist;
    const double normDist  = (distance - minDist) / distRange;

    // Softer proximity score
    const double proximityScore = std::sqrt(1.0 - normDist);

    // Frame separation bonus
    const double frameSep        = static_cast<double>(frameJ - frameI);
    const double maxFrameSep     = static_cast<double>(totalFrames);
    const double separationScore = frameSep / maxFrameSep;

    return 0.6 * proximityScore + 0.4 * separationScore;
}

/** Input arguments for score_multi_objective(). Grouped into a struct to
 *  avoid a long parameter list and to make call sites self-documenting.
 */
struct MultiObjectiveArgs
{
    // Candidate being scored
    double               distance;  // inter-frame distance [m]
    size_t               frameI;
    size_t               frameJ;
    size_t               totalFrames;
    mrpt::math::TPoint3D spatialMidpoint;  // (pose_i + pose_j) / 2

    // Accumulators updated by the greedy selection loop
    const std::vector<double>*               selectedDistances;  // objective 3
    const std::vector<mrpt::math::TPoint3D>* selectedMidpoints;  // objective 4

    // Weights (need not sum to 1; they are normalized internally)
    double wProx;
    double wSep;
    double wDiv;
    double wCov;
};

/**
 * Compute score using multi-objective strategy.
 *
 * Objectives:
 *  1. Proximity     -- prefer spatially close keyframe pairs.
 *  2. Frame sep.    -- prefer temporally distant pairs.
 *  3. Dist. diversity -- penalize pairs whose inter-frame distance is already
 *                        well-represented in the selected set.
 *  4. Spatial coverage -- penalize pairs whose geometric midpoint falls near
 *                         already-selected midpoints, so the set covers
 *                         different map areas.
 */
double score_multi_objective(const MultiObjectiveArgs& a)
{
    // 1. Proximity score
    const double proximityScore = 1.0 / (1.0 + a.distance);

    // 2. Frame separation score
    const auto   frameSep        = static_cast<double>(a.frameJ - a.frameI);
    const auto   maxFrameSep     = static_cast<double>(a.totalFrames);
    const double separationScore = frameSep / maxFrameSep;

    // 3. Distance diversity: penalize inter-frame distances similar to those
    //    already selected. Characteristic scale: 5 m.
    double diversityScore = 1.0;
    for (const double existingDist : *a.selectedDistances)
    {
        const double distDiff = std::abs(a.distance - existingDist);
        const double penalty  = std::exp(-distDiff / 5.0);
        diversityScore *= (1.0 - 0.3 * penalty);
    }

    // 4. Spatial coverage: penalize candidates whose geometric midpoint is
    //    close to an already-selected midpoint, encouraging the set to cover
    //    different map areas. Characteristic scale: 20 m.
    //    Score is 1.0 when no candidates have been selected yet.
    double coverageScore = 1.0;
    for (const auto& existingMidpt : *a.selectedMidpoints)
    {
        const double dx      = a.spatialMidpoint.x - existingMidpt.x;
        const double dy      = a.spatialMidpoint.y - existingMidpt.y;
        const double dz      = a.spatialMidpoint.z - existingMidpt.z;
        const double midDist = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double penalty = std::exp(-midDist / 20.0);
        coverageScore *= (1.0 - 0.5 * penalty);
    }

    // Normalize weights in case they do not sum to 1.0.
    const double wSum = a.wProx + a.wSep + a.wDiv + a.wCov;
    const double w1   = a.wProx / wSum;
    const double w2   = a.wSep / wSum;
    const double w3   = a.wDiv / wSum;
    const double w4   = a.wCov / wSum;

    return w1 * proximityScore + w2 * separationScore + w3 * diversityScore + w4 * coverageScore;
}

std::string first_n_lines(const std::string& input, std::size_t n)
{
    if (n == 0)
    {
        return {};
    }

    std::size_t pos   = 0;
    std::size_t lines = 0;

    while (lines < n)
    {
        pos = input.find('\n', pos);
        if (pos == std::string::npos)
        {
            // Fewer than n lines: return entire string
            return input;
        }
        ++pos;  // move past '\n'
        ++lines;
    }

    return input.substr(0, pos);
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
    YAML_LOAD_OPT(params_, gnss_horizontality_sigma_rpy, double);
    YAML_LOAD_OPT(params_, gnss_edges_uncertainty_multiplier, double);
    YAML_LOAD_OPT(params_, gnss_max_uncertainty_horiz, double);
    YAML_LOAD_OPT(params_, gnss_max_uncertainty_vert, double);

    YAML_LOAD_OPT(params_, use_imu_gravity, bool);
    YAML_LOAD_OPT(params_, imu_gravity_sigma_deg, double);

    YAML_LOAD_OPT(params_, min_distance_between_frames, double);
    YAML_LOAD_OPT(params_, max_distance_for_lc_candidate, double);
    YAML_LOAD_OPT(params_, max_lc_candidates, size_t);
    YAML_LOAD_OPT(params_, min_frames_between_lc, size_t);
    YAML_LOAD_OPT(params_, max_lc_optimization_rounds, size_t);
    YAML_LOAD_OPT(params_, lc_optimize_every_n, size_t);

    YAML_LOAD_OPT(params_, parallel_icp_enabled, bool);
    YAML_LOAD_OPT(params_, num_icp_threads, size_t);
    YAML_LOAD_OPT(params_, deterministic, bool);

    if (params_.min_frames_between_lc == 0)
    {
        MRPT_LOG_WARN("min_frames_between_lc=0 is invalid; clamping to 1.");
        params_.min_frames_between_lc = 1;
    }

    YAML_LOAD_OPT(params_, lc_distance_bins, size_t);
    YAML_LOAD_OPT(params_, lc_weight_proximity, double);
    YAML_LOAD_OPT(params_, lc_weight_frame_separation, double);
    YAML_LOAD_OPT(params_, lc_weight_diversity, double);
    YAML_LOAD_OPT(params_, lc_weight_coverage, double);
    YAML_LOAD_OPT(params_, lc_verbose_candidate_selection, bool);

    // Load enum with string-to-enum conversion
    if (cfg.has("lc_candidate_strategy"))
    {
        params_.lc_candidate_strategy =
            mrpt::typemeta::TEnumType<Parameters::CandidateSelectionStrategy>::name2value(
                cfg["lc_candidate_strategy"].as<std::string>());
    }

    // Validate parameters
    if (params_.lc_distance_bins == 0)
    {
        MRPT_LOG_WARN("lc_distance_bins=0 is invalid; clamping to 1.");
        params_.lc_distance_bins = 1;
    }

    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, min_icp_goodness_to_save_icplog, double);
    YAML_LOAD_OPT(params_, icp_edge_robust_param, double);
    YAML_LOAD_OPT(params_, icp_edge_additional_noise_xyz, double);
    YAML_LOAD_OPT(params_, icp_edge_additional_noise_ang, double);
    YAML_LOAD_OPT(params_, threshold_sigma_initial, std::string);
    YAML_LOAD_OPT(params_, threshold_sigma_final, std::string);

    YAML_LOAD_OPT(params_, input_odometry_noise_xyz, double);
    YAML_LOAD_OPT(params_, input_odometry_noise_ang, double);
    YAML_LOAD_OPT(params_, scale_odometry_noise_by_distance, bool);

    YAML_LOAD_OPT(params_, pc_cache_max_bytes, size_t);
    YAML_LOAD_OPT(params_, unload_observations_after_use, bool);

    YAML_LOAD_OPT(params_, assume_planar_world, bool);
    YAML_LOAD_OPT(params_, planar_world_initial_sigma_z, double);
    YAML_LOAD_OPT(params_, planar_world_initial_sigma_ang, double);
    YAML_LOAD_OPT(params_, planar_world_annealing_rounds, size_t);

    YAML_LOAD_OPT(params_, use_kiss_matcher, bool);
    YAML_LOAD_OPT(params_, kiss_matcher_resolution, double);
    YAML_LOAD_OPT(params_, kiss_matcher_layer, std::string);
    YAML_LOAD_OPT(params_, kiss_matcher_min_inliers, uint32_t);

    YAML_LOAD_OPT(params_, largest_delta_for_reconsider, double);
    YAML_LOAD_OPT(params_, max_sensor_range, double);

    YAML_LOAD_OPT(params_, profiler_enabled, bool);
    YAML_LOAD_OPT(params_, save_trajectory_files, bool);
    YAML_LOAD_OPT(params_, save_trajectory_files_with_cov, bool);
    YAML_LOAD_OPT(params_, debug_files_prefix, std::string);

    YAML_LOAD_OPT(params_, save_3d_scene_files, bool);
    YAML_LOAD_OPT(params_, save_3d_scene_files_per_iteration, bool);
    YAML_LOAD_OPT(params_, save_3d_scene_live_preview, bool);
    YAML_LOAD_OPT(params_, scene_path_line_width, float);
    YAML_LOAD_OPT(params_, scene_lc_line_width, float);
    YAML_LOAD_OPT(params_, scene_path_color_r, float);
    YAML_LOAD_OPT(params_, scene_path_color_g, float);
    YAML_LOAD_OPT(params_, scene_path_color_b, float);
    YAML_LOAD_OPT(params_, scene_path_color_a, float);
    YAML_LOAD_OPT(params_, scene_lc_color_r, float);
    YAML_LOAD_OPT(params_, scene_lc_color_g, float);
    YAML_LOAD_OPT(params_, scene_lc_color_b, float);
    YAML_LOAD_OPT(params_, scene_lc_color_a, float);
    YAML_LOAD_OPT(params_, scene_lc_candidate_color_r, float);
    YAML_LOAD_OPT(params_, scene_lc_candidate_color_g, float);
    YAML_LOAD_OPT(params_, scene_lc_candidate_color_b, float);
    YAML_LOAD_OPT(params_, scene_lc_candidate_color_a, float);
    YAML_LOAD_OPT(params_, scene_keyframe_point_size, float);

    // Load manual loop closure hints
    if (cfg.has("manual_loop_constraints") && !cfg["manual_loop_constraints"].isNullNode())
    {
        for (const auto& entryNode : cfg["manual_loop_constraints"].asSequenceRange())
        {
            ASSERT_(entryNode.isMap());
            const mrpt::containers::yaml entry(entryNode);

            Parameters::ManualLoopConstraint mlc;
            ASSERTMSG_(
                entry.has("timestamp_i") && entry.has("timestamp_j") && entry.has("sigma_xyz"),
                "Each manual_loop_constraints entry must have: timestamp_i, timestamp_j, "
                "sigma_xyz");

            mlc.timestamp_i = entry["timestamp_i"].as<double>();
            mlc.timestamp_j = entry["timestamp_j"].as<double>();
            mlc.sigma_xyz   = entry["sigma_xyz"].as<double>();
            if (entry.has("trust_as_inlier"))
                mlc.trust_as_inlier = entry["trust_as_inlier"].as<bool>();

            params_.manual_loop_constraints.push_back(mlc);
        }
        MRPT_LOG_INFO_STREAM(
            "Loaded " << params_.manual_loop_constraints.size()
                      << " manual loop closure constraint(s) from config.");
    }

    profiler_.enable(params_.profiler_enabled);

    // Initialize ICP pipelines for each thread
    for (auto& pts : state_.perThreadState_)
    {
        params_.icp_parameters = lc_common::load_icp_pipeline_from_yaml(
            c, pts.pipeline, params_.threshold_sigma_initial, params_.threshold_sigma_final);
    }

#if MP2P_ICP_HAS_LOG_FUNCTOR  // MP2P_ICP>=2.6.0
    //  Only generate log files for good ICP edges:
    params_.icp_parameters.functor_should_generate_debug_file =
        [this](const mp2p_icp::LogRecord& log) -> bool
    {
        return params_.icp_parameters.generateDebugFiles &&
               log.icpResult.quality >= params_.min_icp_goodness_to_save_icplog;
    };
#endif

#ifdef MOLA_HAS_KISS_MATCHER
    if (params_.use_kiss_matcher)
    {
        const kiss_matcher::KISSMatcherConfig km_cfg(
            static_cast<float>(params_.kiss_matcher_resolution));
        for (auto& pts : state_.perThreadState_)
            pts.kissMatcher = std::make_shared<kiss_matcher::KISSMatcher>(km_cfg);
        MRPT_LOG_INFO_STREAM(
            "KISS-Matcher enabled: resolution=" << params_.kiss_matcher_resolution << " m, layer='"
                                                << params_.kiss_matcher_layer << "'");
    }
#else
    if (params_.use_kiss_matcher)
    {
        MRPT_LOG_WARN(
            "use_kiss_matcher=true but this build lacks KISS-Matcher support "
            "(populate the third_party/kiss-matcher submodule and rebuild); ignoring.");
    }
#endif

    state_.initialized = true;

    MRPT_TRY_END
}

void FrameToFrameLoopClosure::process(mrpt::maps::CSimpleMap& sm)  // NOLINT
{
    using namespace std::string_literals;

    ASSERT_(state_.initialized);

    // Same contract as analyze(): with `deterministic` set, this whole batch
    // pass -- every ICP inside it, and every parallel runtime underneath --
    // runs on one thread, so the corrected map is a function of the input.
    const DeterministicScope detScope{params_.deterministic, this};

    state_.sm               = &sm;
    state_.readOnlySnapshot = false;
    state_.pcCacheClear();
    accepted_lc_edges_.clear();

    MRPT_LOG_INFO_STREAM(
        "Processing simplemap with " << sm.size() << " frames"
                                     << (params_.deterministic ? " [deterministic]" : ""));

    // Precompute which frames have mapping-capable observations, so that
    // find_loop_candidates() does not need to access (and lazy-load) the
    // raw sensory frames on every O(N^2) candidate pair check.
    {
        state_.frameHasMappingObs.assign(sm.size(), false);
        for (size_t i = 0; i < sm.size(); i++)
        {
            const auto& kf               = sm.get(i);
            state_.frameHasMappingObs[i] = kf.sf && frame_has_mapping_observations(*kf.sf);
            if (params_.unload_observations_after_use && kf.sf)
            {
                for (const auto& obs : *kf.sf)
                {
                    obs->unload();
                }
            }
        }
    }

    // Build initial graph with odometry edges
    build_initial_graph();

    // Seed planarity constraint at full strength before any optimization
    if (params_.assume_planar_world)
    {
        build_planarity_factors(
            params_.planar_world_initial_sigma_z, params_.planar_world_initial_sigma_ang);
        MRPT_LOG_INFO_STREAM(
            "Planar-world annealing enabled: initial sigma_z="
            << params_.planar_world_initial_sigma_z
            << " m, sigma_ang=" << params_.planar_world_initial_sigma_ang << " rad, over "
            << params_.planar_world_annealing_rounds << " LC rounds");
    }

    if (params_.save_trajectory_files)
    {
        optimize_graph();

        save_trajectory_as_tum(
            params_.debug_files_prefix + "initial.tum"s, params_.save_trajectory_files_with_cov);
    }

    // Add GNSS factors if available
    if (params_.use_gnss)
    {
        add_gnss_factors();

        // Initial optimization with GNSS

        MRPT_LOG_INFO("Running initial GNSS optimization...");
        optimize_graph();

        if (params_.save_trajectory_files)
        {
            save_trajectory_as_tum(
                params_.debug_files_prefix + "after_gnss.tum"s,
                params_.save_trajectory_files_with_cov);
        }
    }

    // Add IMU gravity-alignment factors if requested (works with or without GNSS)
    if (params_.use_imu_gravity && add_imu_gravity_factors())
    {
        MRPT_LOG_INFO("Running optimization after IMU gravity-alignment factors...");
        optimize_graph();

        if (params_.save_trajectory_files)
        {
            save_trajectory_as_tum(
                params_.debug_files_prefix + "after_imu_gravity.tum"s,
                params_.save_trajectory_files_with_cov);
        }
    }

    // Add manual loop closure constraints, if any
    if (!params_.manual_loop_constraints.empty())
    {
        add_manual_loop_closure_factors();
        MRPT_LOG_INFO("Running optimization after manual loop closure constraints...");
        optimize_graph();

        if (params_.save_trajectory_files)
        {
            save_trajectory_as_tum(
                params_.debug_files_prefix + "after_manual_lc.tum"s,
                params_.save_trajectory_files_with_cov);
        }
    }

    if (params_.save_3d_scene_files)
    {
        save_3d_scene_initial_files();
    }

    // Loop closure detection and optimization
    size_t                                      accepted_lcs    = 0;
    size_t                                      lastGncInliers  = 0;
    size_t                                      lastGncOutliers = 0;
    std::set<std::pair<frame_id_t, frame_id_t>> alreadyChecked;

    for (size_t lcRound = 0; lcRound < params_.max_lc_optimization_rounds; lcRound++)
    {
        // Anneal planar-world constraint: grows from initial sigma to 1e6 over
        // planar_world_annealing_rounds, then the constraint is dropped entirely.
        if (params_.assume_planar_world)
        {
            const size_t N = params_.planar_world_annealing_rounds;
            if (lcRound >= N)
            {
                state_.planarityFG.resize(0);
                if (lcRound == N)
                {
                    MRPT_LOG_INFO("Planar-world constraint fully annealed out.");
                }
            }
            else
            {
                lc_common::PlanarAnneal pa;
                pa.rounds         = N;
                pa.initSigmaZ     = params_.planar_world_initial_sigma_z;
                pa.initSigmaAng   = params_.planar_world_initial_sigma_ang;
                const auto sigmas = lc_common::planar_sigmas_for_round(pa, lcRound);
                ASSERT_(sigmas.has_value());
                const auto [sigmaZ, sigmaAng] = *sigmas;
                build_planarity_factors(sigmaZ, sigmaAng);
                MRPT_LOG_INFO_STREAM(
                    "Planar-world round " << lcRound << "/" << N << ": sigma_z=" << sigmaZ
                                          << " m, sigma_ang=" << sigmaAng << " rad");
            }
        }

        size_t checkedCount   = 0;
        bool   anyGraphChange = false;

        auto candidates = find_loop_candidates(alreadyChecked);

        MRPT_LOG_INFO_STREAM("Found " << candidates.size() << " loop closure candidates");

        // Sort candidates by ascending topological gap (frame index separation)
        // so that inner (smaller) loops are closed first, improving the graph
        // before attempting larger loops:
        std::sort(
            candidates.begin(), candidates.end(),
            [](const LoopCandidate& a, const LoopCandidate& b)
            {
                const auto gapA = a.frame_j - a.frame_i;
                const auto gapB = b.frame_j - b.frame_i;
                if (gapA != gapB)
                {
                    return gapA < gapB;
                }
                // Tie-break: earlier frame first (cache locality)
                return std::min(a.frame_i, a.frame_j) < std::min(b.frame_i, b.frame_j);
            });

        const auto frameGroup           = static_cast<double>(params_.min_frames_between_lc);
        size_t     acceptedSinceLastOpt = 0;

        LivePreviewStats liveStats;
        liveStats.lcRound         = lcRound;
        liveStats.totalRounds     = params_.max_lc_optimization_rounds;
        liveStats.acceptedLCs     = accepted_lcs;
        liveStats.candidatesTotal = candidates.size();
        liveStats.candidatesDone  = 0;
        // Carry forward GNC stats from previous rounds so the live preview
        // caption does not reset to "0 inliers, 0 outliers" at each round start.
        liveStats.gncInliers  = lastGncInliers;
        liveStats.gncOutliers = lastGncOutliers;

        if (params_.save_3d_scene_live_preview)
        {
            save_3d_scene_live_preview(candidates, liveStats);
        }

        for (size_t ci = 0; ci < candidates.size(); ci++)
        {
            const auto& lc = candidates[ci];

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
            liveStats.candidatesDone = checkedCount;

            const auto lc_result = process_loop_candidate(lc);
            if (lc_result.has_value())
            {
                anyGraphChange = true;
                accepted_lcs++;
                acceptedSinceLastOpt++;

                if (params_.save_3d_scene_live_preview)
                {
                    const std::vector<LoopCandidate> remaining(
                        candidates.begin() + ci + 1, candidates.end());
                    liveStats.acceptedLCs    = accepted_lcs;
                    liveStats.candidatesDone = checkedCount;
                    save_3d_scene_live_preview(remaining, liveStats);
                }

                // Intermediate optimization: re-optimize after every N accepted LCs
                // so that later (larger-gap) candidates benefit from corrected poses.
                if (params_.lc_optimize_every_n > 0 &&
                    acceptedSinceLastOpt >= params_.lc_optimize_every_n)
                {
                    MRPT_LOG_INFO_STREAM(
                        "Intermediate optimization after " << acceptedSinceLastOpt
                                                           << " accepted LCs");
                    {
                        const auto optRes     = optimize_graph();
                        liveStats.gncInliers  = optRes.numLcInliers;
                        liveStats.gncOutliers = optRes.numLcOutliers;
                        lastGncInliers        = optRes.numLcInliers;
                        lastGncOutliers       = optRes.numLcOutliers;
                    }
                    acceptedSinceLastOpt = 0;

                    if (params_.save_3d_scene_live_preview)
                    {
                        const std::vector<LoopCandidate> remaining(
                            candidates.begin() + ci + 1, candidates.end());
                        liveStats.acceptedLCs    = accepted_lcs;
                        liveStats.candidatesDone = checkedCount;
                        save_3d_scene_live_preview(remaining, liveStats);
                    }
                }
            }
        }

        if (checkedCount == 0)
        {
            break;  // No new candidates
        }

        if (anyGraphChange && acceptedSinceLastOpt > 0)
        {
            // Final optimization for remaining accepted LCs in this round
            const auto   optRes       = optimize_graph();
            const double largestDelta = optRes.largestDelta;
            liveStats.gncInliers      = optRes.numLcInliers;
            liveStats.gncOutliers     = optRes.numLcOutliers;
            lastGncInliers            = optRes.numLcInliers;
            lastGncOutliers           = optRes.numLcOutliers;

            if (params_.save_3d_scene_files && params_.save_3d_scene_files_per_iteration)
            {
                save_3d_scene_files(mrpt::format("_iter%02zu", lcRound));
            }

            if (largestDelta > params_.largest_delta_for_reconsider)
            {
                MRPT_LOG_INFO_STREAM(
                    "Large pose change detected (" << largestDelta
                                                   << "m), reconsidering all candidates");
                alreadyChecked.clear();
            }
        }

        if (params_.save_3d_scene_live_preview && checkedCount > 0)
        {
            liveStats.acceptedLCs    = accepted_lcs;
            liveStats.candidatesDone = checkedCount;
            save_3d_scene_live_preview({}, liveStats);
        }
    }

    MRPT_LOG_INFO_STREAM(
        "Total accepted loop closures: " << accepted_lcs << " (GNC: " << lastGncInliers
                                         << " inliers, " << lastGncOutliers
                                         << " outliers rejected)");

    if (params_.save_trajectory_files)
    {
        save_trajectory_as_tum(params_.debug_files_prefix + "final.tum"s);
    }

    if (params_.save_3d_scene_files)
    {
        save_3d_scene_files();
    }

    // Update simplemap with optimized poses
    using gtsam::symbol_shorthand::X;
    mrpt::maps::CSimpleMap outSM;
    for (size_t id = 0; id < sm.size(); id++)
    {
        auto& [oldPose, sf, twist] = sm.get(id);

        const auto newPose = mrpt::poses::CPose3DPDFGaussian::Create();
        newPose->mean      = state_.get_pose(id);
        if (state_.graphMarginals.has_value())
        {
            try
            {
                newPose->cov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(
                    state_.graphMarginals->marginalCovariance(X(id)));
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_WARN_STREAM(
                    "[f2f_lc] Marginal covariance unavailable for frame " << id << ": "
                                                                          << e.what());
                newPose->cov.setIdentity();
            }
        }
        else
        {
            newPose->cov.setIdentity();
        }

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

    // Start from a pristine graph: gtsam Values::insert() throws on duplicate
    // keys, so guard against stale state left by a prior analyze()/process().
    state_.graphValues.clear();
    state_.graphFG.resize(0);

    // Add all frame poses to values
    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto pose_i = frame_pose_in_simplemap(i);
        state_.graphValues.insert(X(i), mrpt::gtsam_wrappers::toPose3(pose_i));
    }

    // Track known inlier factor indices for GNC optimizer
    state_.knownInlierFactorIndices.clear();

    // Add prior on first frame: very weak, so GNSS can override it as needed.
    // if not using GNSS, let X(0) be anchored.
    const double priorSigma = params_.use_gnss ? 1e+2 : 1e-2;

    const auto pose0      = frame_pose_in_simplemap(0);
    auto       priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, priorSigma);

    state_.knownInlierFactorIndices.push_back(state_.graphFG.size());
    state_.graphFG.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(0), mrpt::gtsam_wrappers::toPose3(pose0), priorNoise);

    // Add odometry edges between consecutive frames
    for (size_t i = 1; i < sm.size(); i++)
    {
        const auto pose_i   = frame_pose_in_simplemap(i);
        const auto pose_im1 = frame_pose_in_simplemap(i - 1);

        const auto relPose   = pose_i - pose_im1;
        const auto deltaPose = mrpt::gtsam_wrappers::toPose3(relPose);

        // Scale noise by inter-frame distance
        const double dist = relPose.translation().norm();
        const double distScale =
            params_.scale_odometry_noise_by_distance ? std::max(1.0, dist) : 1.0;

        const double noiseXyz = params_.input_odometry_noise_xyz * distScale;
        const double noiseAng = params_.input_odometry_noise_ang * distScale;

        gtsam::Vector6 sigmas;
        sigmas << mrpt::DEG2RAD(noiseAng), mrpt::DEG2RAD(noiseAng), mrpt::DEG2RAD(noiseAng),
            noiseXyz, noiseXyz, noiseXyz;

        auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

        state_.knownInlierFactorIndices.push_back(state_.graphFG.size());

#if GTSAM_USES_BOOST
        auto factor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);
#else
        auto factor = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);
#endif
        state_.graphFG += factor;
    }

    MRPT_LOG_INFO_STREAM("Built initial graph with " << sm.size() << " frames");
}

void FrameToFrameLoopClosure::add_gnss_factors()
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "add_gnss_factors");

    ASSERT_(state_.sm);

    lc_common::GnssFactorParams p;
    p.add_horizontality       = params_.gnss_add_horizontality;
    p.horizontality_sigma_rpy = params_.gnss_horizontality_sigma_rpy;
    p.minimum_uncertainty_xyz = params_.gnss_minimum_uncertainty_xyz;
    p.uncertainty_multiplier  = params_.gnss_edges_uncertainty_multiplier;
    p.max_uncertainty_horiz   = params_.gnss_max_uncertainty_horiz;
    p.max_uncertainty_vert    = params_.gnss_max_uncertainty_vert;

    lc_common::add_gnss_factors_per_kf(
        state_.graphFG, *state_.sm, state_.globalGeoRef, p, state_.knownInlierFactorIndices, this);
}

bool FrameToFrameLoopClosure::add_imu_gravity_factors()
{
    using gtsam::symbol_shorthand::T;
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "add_imu_gravity_factors");

    ASSERT_(state_.sm);

#ifdef MOLA_GEOREFERENCING_HAS_NEW_IMU_API
    const auto imuFrames = mola::extract_imu_frames_from_sm(*state_.sm);
#else
    const auto imuFrames = mola::extract_imu_acc_frames_from_sm(*state_.sm);
#endif
    if (imuFrames.frames.empty())
    {
        MRPT_LOG_WARN(
            "use_imu_gravity=true but no per-keyframe IMU accelerometer data was found in the "
            "input simplemap; skipping IMU gravity-alignment factors.");
        return false;
    }

    // T(0) is a fixed anchor for the ENU->map transform, as required by
    // MeasuredGravityFactor's signature. Frame poses here already live in the map frame (X(i)
    // keys, see build_initial_graph()), so T(0) is simply locked at Identity.
    if (!state_.graphValues.exists(T(0)))
    {
        state_.graphValues.insert(T(0), gtsam::Pose3::Identity());
        auto tightNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
        state_.knownInlierFactorIndices.push_back(state_.graphFG.size());
        state_.graphFG.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            T(0), gtsam::Pose3::Identity(), tightNoise);
    }

    ASSERTMSG_(
        params_.imu_gravity_sigma_deg > 0,
        "params_.imu_gravity_sigma_deg must be a positive angle in degrees");

    auto accNoise =
        gtsam::noiseModel::Isotropic::Sigma(3, mrpt::DEG2RAD(params_.imu_gravity_sigma_deg));

    const std::size_t before = state_.graphFG.size();
    for (const auto& frame : imuFrames.frames)
    {
#ifdef MOLA_GEOREFERENCING_HAS_NEW_IMU_API
        if (!frame.normalizedAcc)
        {
            continue;
        }
        const auto& normalizedAcc = *frame.normalizedAcc;
#else
        const auto& normalizedAcc = frame.normalizedAcc;
#endif
        const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(frame.sensorPoseOnVehicle);
        state_.knownInlierFactorIndices.push_back(state_.graphFG.size());
        state_.graphFG.emplace_shared<mola::factors::MeasuredGravityFactor>(
            T(0), X(frame.kf_index), sensorOnVehicle, normalizedAcc, accNoise);
    }
    const std::size_t after = state_.graphFG.size();
    MRPT_LOG_INFO_STREAM(
        "Added " << (after - before) << " IMU gravity factors over " << imuFrames.frames.size()
                 << " IMU keyframes");

    return after > before;
}

void FrameToFrameLoopClosure::add_manual_loop_closure_factors()
{
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "add_manual_loop_closure_factors");

    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    // Build a timestamp => frame_id lookup table once
    // (timestamps come from the first observation in each sensory frame)
    std::vector<std::pair<double, frame_id_t>> tsIndex;
    tsIndex.reserve(sm.size());

    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto& kf = sm.get(i);
        if (!kf.sf || kf.sf->empty())
        {
            continue;
        }
        const auto obs = kf.sf->getObservationByIndex(0);
        if (!obs)
        {
            continue;
        }
        tsIndex.emplace_back(mrpt::Clock::toDouble(obs->timestamp), static_cast<frame_id_t>(i));
    }

    // Sort by timestamp for fast nearest-neighbour lookup
    std::sort(tsIndex.begin(), tsIndex.end());

    // Helper: find the frame_id whose timestamp is closest to a query value
    auto findClosestFrame = [&](double queryTs) -> std::optional<frame_id_t>
    {
        if (tsIndex.empty())
        {
            return std::nullopt;
        }

        // Lower bound by timestamp
        auto it = std::lower_bound(
            tsIndex.begin(), tsIndex.end(), std::make_pair(queryTs, frame_id_t{0}));

        if (it == tsIndex.end())
        {
            return tsIndex.back().second;
        }
        if (it == tsIndex.begin())
        {
            return it->second;
        }

        auto prev = std::prev(it);
        return (std::abs(it->first - queryTs) < std::abs(prev->first - queryTs)) ? it->second
                                                                                 : prev->second;
    };

    size_t addedCount = 0;

    for (const auto& mlc : params_.manual_loop_constraints)
    {
        const auto fi_opt = findClosestFrame(mlc.timestamp_i);
        const auto fj_opt = findClosestFrame(mlc.timestamp_j);

        if (!fi_opt || !fj_opt)
        {
            MRPT_LOG_WARN("Manual LC: could not find frames for the given timestamps; skipping.");
            continue;
        }

        const frame_id_t fi = *fi_opt;
        const frame_id_t fj = *fj_opt;

        if (fi == fj)
        {
            MRPT_LOG_WARN_STREAM(
                "Manual LC: timestamps map to the same frame (" << fi << "); skipping.");
            continue;
        }

#ifdef MOLA_HAS_KISS_MATCHER
        // When KISS-Matcher is available, use it (+ ICP) to compute the actual
        // relative pose rather than assuming an identity transform.
        if (params_.use_kiss_matcher)
        {
            LoopCandidate lc;
            lc.frame_i = fi;
            lc.frame_j = fj;
            lc.distance =
                (state_.get_pose(fi).translation() - state_.get_pose(fj).translation()).norm();
            lc.score = 1.0;

            if (const auto km_result = process_loop_candidate(lc); km_result.has_value())
            {
                if (mlc.trust_as_inlier) state_.knownInlierFactorIndices.push_back(*km_result);
                addedCount++;
                MRPT_LOG_INFO_STREAM(
                    "Manual LC (KISS-Matcher+ICP) added: frame "
                    << fi << " (t=" << mlc.timestamp_i << ") <-> frame " << fj
                    << " (t=" << mlc.timestamp_j << ")");
                continue;
            }
            MRPT_LOG_WARN_STREAM(
                "Manual LC: KISS-Matcher+ICP failed for frames "
                << fi << "<->" << fj
                << " (ICP quality too low); falling back to identity constraint "
                   "with sigma_xyz="
                << mlc.sigma_xyz << " m");
        }
#endif

        // Fallback (or when KISS-Matcher is disabled): identity pose constraint
        // with a tight XYZ sigma and unconstrained angles.
        const auto deltaPose = gtsam::Pose3::Identity();

        // Tight sigma on XYZ, very loose on angles (leave orientation free)
        constexpr double LARGE_ANGLE_SIGMA = 1e3;  // [rad] effectively unconstrained
        gtsam::Vector6   sigmas;
        // GTSAM Pose3 noise order: rx, ry, rz, tx, ty, tz
        sigmas << LARGE_ANGLE_SIGMA, LARGE_ANGLE_SIGMA, LARGE_ANGLE_SIGMA, mlc.sigma_xyz,
            mlc.sigma_xyz, mlc.sigma_xyz;

        auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

        if (mlc.trust_as_inlier) state_.knownInlierFactorIndices.push_back(state_.graphFG.size());

#if GTSAM_USES_BOOST
        auto factor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(fi), X(fj), deltaPose, edgeNoise);
#else
        auto        factor        = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(fi), X(fj), deltaPose, edgeNoise);
#endif
        state_.graphFG += factor;

        // Track for 3D scene output
        accepted_lc_edges_.emplace_back(fi, fj);
        addedCount++;

        MRPT_LOG_INFO_STREAM(
            "Manual LC (identity) added: frame "
            << fi << " (t=" << mlc.timestamp_i << ") <-> frame " << fj << " (t=" << mlc.timestamp_j
            << ")  sigma_xyz=" << mlc.sigma_xyz << " m");
    }

    MRPT_LOG_INFO_STREAM("Added " << addedCount << " manual loop closure factor(s).");
}

auto FrameToFrameLoopClosure::
    find_loop_candidates(  // NOLINT(readability-function-cognitive-complexity)
        const std::set<std::pair<frame_id_t, frame_id_t>>& alreadyChecked,
        frame_id_t minLaterFrame) const -> std::vector<FrameToFrameLoopClosure::LoopCandidate>
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "find_loop_candidates");

    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    const auto   frameGroup = static_cast<double>(params_.min_frames_between_lc);
    const double minDist    = params_.min_distance_between_frames;
    const double maxDist    = params_.max_distance_for_lc_candidate;

    // For multi-objective strategy: track selected distances and spatial
    // midpoints so the greedy selection step can update scores incrementally.
    std::vector<double>               selectedDistances;
    std::vector<mrpt::math::TPoint3D> selectedMidpoints;

    // Determine if we need distance binning
    const bool useStratification =
        (params_.lc_candidate_strategy ==
         Parameters::CandidateSelectionStrategy::DISTANCE_STRATIFIED);

    // Setup distance bins if using stratified strategy
    std::vector<std::vector<LoopCandidate>> binnedCandidates;
    double                                  binWidth = 0.0;
    if (useStratification)
    {
        binnedCandidates.resize(params_.lc_distance_bins);
        binWidth = (maxDist - minDist) / static_cast<double>(params_.lc_distance_bins);
    }

    // Single vector for non-stratified approaches
    std::vector<LoopCandidate> candidates;

    // ========================================================================
    // STEP 1: Generate and score all candidates
    // ========================================================================

    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto pose_i = state_.get_pose(i);

        // Incremental scans skip pairs whose later frame predates the newest
        // batch (both endpoints already evaluated in a previous call).
        const size_t jStart = std::max<size_t>(i + params_.min_frames_between_lc, minLaterFrame);

        for (size_t j = jStart; j < sm.size(); j++)
        {
            // Check if already evaluated
            const auto frameGroup_i = mrpt::round(static_cast<double>(i) / frameGroup);
            const auto frameGroup_j = mrpt::round(static_cast<double>(j) / frameGroup);

            const auto IDs = std::make_pair(
                std::min<frame_id_t>(frameGroup_i, frameGroup_j),
                std::max<frame_id_t>(frameGroup_i, frameGroup_j));

            if (alreadyChecked.count(IDs) != 0)
            {
                continue;
            }

            // Compute spatial distance
            const auto   pose_j   = state_.get_pose(j);
            const double distance = (pose_i.translation() - pose_j.translation()).norm();

            // Apply distance constraints
            if (distance < minDist || distance > maxDist)
            {
                continue;
            }

            // Verify valid observations (using precomputed flags to avoid
            // lazy-loading externally-stored observation data)
            if (!state_.frameHasMappingObs[i] || !state_.frameHasMappingObs[j])
            {
                continue;
            }

            // Create candidate
            LoopCandidate lc;
            lc.frame_i  = i;
            lc.frame_j  = j;
            lc.distance = distance;

            // Precompute spatial midpoint (used by MULTI_OBJECTIVE coverage term).
            const auto ti      = pose_i.translation();
            const auto tj      = pose_j.translation();
            lc.spatialMidpoint = {0.5 * (ti.x + tj.x), 0.5 * (ti.y + tj.y), 0.5 * (ti.z + tj.z)};

            // Compute score based on selected strategy
            switch (params_.lc_candidate_strategy)
            {
                case Parameters::CandidateSelectionStrategy::PROXIMITY_ONLY:
                    lc.score = score_proximity_only(distance);
                    break;

                case Parameters::CandidateSelectionStrategy::DISTANCE_STRATIFIED:
                    lc.score = score_stratified(distance, minDist, maxDist, i, j, sm.size());
                    break;

                case Parameters::CandidateSelectionStrategy::MULTI_OBJECTIVE:
                {
                    // selectedDistances and selectedMidpoints are empty here;
                    // the greedy step in STEP 2 will update them and re-score.
                    MultiObjectiveArgs args;
                    args.distance          = distance;
                    args.frameI            = i;
                    args.frameJ            = j;
                    args.totalFrames       = sm.size();
                    args.spatialMidpoint   = lc.spatialMidpoint;
                    args.selectedDistances = &selectedDistances;
                    args.selectedMidpoints = &selectedMidpoints;
                    args.wProx             = params_.lc_weight_proximity;
                    args.wSep              = params_.lc_weight_frame_separation;
                    args.wDiv              = params_.lc_weight_diversity;
                    args.wCov              = params_.lc_weight_coverage;
                    lc.score               = score_multi_objective(args);
                    break;
                }
            }

            // Add to appropriate container
            if (useStratification)
            {
                // Determine bin index
                const size_t binIdx = std::min(
                    static_cast<size_t>((distance - minDist) / binWidth),
                    params_.lc_distance_bins - 1);
                binnedCandidates[binIdx].push_back(lc);
            }
            else
            {
                candidates.push_back(lc);
            }

            if (PRINT_LC_SCORES)
            {
                MRPT_LOG_DEBUG_STREAM(
                    "Candidate: " << i << " <-> " << j << " dist=" << distance
                                  << " score=" << lc.score);
            }
        }
    }

    // ========================================================================
    // STEP 1b: Deduplicate by block-pair ID within this call.
    //
    // find_loop_candidates() filters out block-pairs already in alreadyChecked
    // (from previous rounds), but multiple (i,j) pairs can round to the same
    // (frameGroup_i, frameGroup_j) block and all pass that check.  If more than
    // one such pair survives into the final candidate list, the evaluation loop
    // in process() will skip all but the first (because it inserts the block-pair
    // into alreadyChecked after the first evaluation).  Deduplicate here --
    // keeping the highest-scored candidate per block pair -- so that
    // max_lc_candidates truly reflects the number that will be evaluated.
    {
        auto dedup = [&frameGroup](std::vector<LoopCandidate>& vec)
        {
            std::map<std::pair<frame_id_t, frame_id_t>, size_t> best;  // blockPair -> index
            for (size_t k = 0; k < vec.size(); k++)
            {
                const auto bg_i = static_cast<frame_id_t>(
                    mrpt::round(static_cast<double>(vec[k].frame_i) / frameGroup));
                const auto bg_j = static_cast<frame_id_t>(
                    mrpt::round(static_cast<double>(vec[k].frame_j) / frameGroup));
                const auto key = std::make_pair(std::min(bg_i, bg_j), std::max(bg_i, bg_j));
                auto       it  = best.find(key);
                if (it == best.end() || vec[k].score > vec[it->second].score)
                {
                    best[key] = k;
                }
            }
            std::vector<LoopCandidate> out;
            out.reserve(best.size());
            for (const auto& kv : best)
            {
                out.push_back(vec[kv.second]);
            }
            vec = std::move(out);
        };

        if (useStratification)
        {
            for (auto& bin : binnedCandidates)
            {
                dedup(bin);
            }
        }
        else
        {
            dedup(candidates);
        }
    }

    // ========================================================================
    // STEP 2: Select final candidates based on strategy
    // ========================================================================

    std::vector<LoopCandidate> finalCandidates;

    if (useStratification)
    {
        // Strategy: Sample proportionally from each distance bin

        const size_t baseCandidatesPerBin = params_.max_lc_candidates / params_.lc_distance_bins;
        const size_t extraCandidates      = params_.max_lc_candidates % params_.lc_distance_bins;

        for (size_t binIdx = 0; binIdx < params_.lc_distance_bins; binIdx++)
        {
            auto& bin = binnedCandidates[binIdx];

            if (bin.empty())
            {
                continue;
            }

            // Sort within bin
            std::sort(
                bin.begin(), bin.end(),
                [](const LoopCandidate& a, const LoopCandidate& b) { return a.score > b.score; });

            // Determine number to select from this bin
            size_t toTake = baseCandidatesPerBin;
            if (binIdx < extraCandidates)
            {
                toTake++;
            }
            toTake = std::min(toTake, bin.size());

            // Add top candidates from bin
            for (size_t k = 0; k < toTake; k++)
            {
                finalCandidates.push_back(bin[k]);
            }

            if (params_.lc_verbose_candidate_selection)
            {
                const double binMin = minDist + static_cast<double>(binIdx) * binWidth;
                const double binMax = minDist + static_cast<double>(binIdx + 1) * binWidth;
                MRPT_LOG_INFO_STREAM(
                    "Bin [" << binMin << ", " << binMax << "] m: " << bin.size()
                            << " candidates, selected " << toTake);
            }
        }

        // Final global sort
        std::sort(
            finalCandidates.begin(), finalCandidates.end(),
            [](const LoopCandidate& a, const LoopCandidate& b) { return a.score > b.score; });
    }
    else if (
        params_.lc_candidate_strategy == Parameters::CandidateSelectionStrategy::MULTI_OBJECTIVE)
    {
        // Greedy diversity-aware selection:
        //   1. Pick the highest-scored remaining candidate.
        //   2. Record its distance and spatial midpoint.
        //   3. Re-score all remaining candidates so the diversity and coverage
        //      terms now penalize already-represented distances and map areas.
        //   4. Repeat until max_lc_candidates are chosen or none remain.
        //
        // Scoring all candidates upfront with empty accumulators would make
        // the diversity and coverage objectives no-ops, so we do it here.
        while (finalCandidates.size() < params_.max_lc_candidates && !candidates.empty())
        {
            auto bestIt = std::max_element(
                candidates.begin(), candidates.end(),
                [](const LoopCandidate& a, const LoopCandidate& b) { return a.score < b.score; });

            finalCandidates.push_back(*bestIt);
            selectedDistances.push_back(bestIt->distance);
            selectedMidpoints.push_back(bestIt->spatialMidpoint);
            candidates.erase(bestIt);

            // Re-score remaining candidates with updated accumulators.
            for (auto& lc : candidates)
            {
                MultiObjectiveArgs args;
                args.distance          = lc.distance;
                args.frameI            = lc.frame_i;
                args.frameJ            = lc.frame_j;
                args.totalFrames       = sm.size();
                args.spatialMidpoint   = lc.spatialMidpoint;
                args.selectedDistances = &selectedDistances;
                args.selectedMidpoints = &selectedMidpoints;
                args.wProx             = params_.lc_weight_proximity;
                args.wSep              = params_.lc_weight_frame_separation;
                args.wDiv              = params_.lc_weight_diversity;
                args.wCov              = params_.lc_weight_coverage;
                lc.score               = score_multi_objective(args);
            }
        }
    }
    else
    {
        // PROXIMITY_ONLY: simple top-K selection by score.
        std::sort(
            candidates.begin(), candidates.end(),
            [](const LoopCandidate& a, const LoopCandidate& b) { return a.score > b.score; });

        finalCandidates = std::move(candidates);
    }

    // Limit to max candidates
    if (finalCandidates.size() > params_.max_lc_candidates)
    {
        finalCandidates.resize(params_.max_lc_candidates);
    }

    // ========================================================================
    // STEP 3: Log statistics (if verbose or always at INFO level)
    // ========================================================================

    if (!finalCandidates.empty() && (params_.lc_verbose_candidate_selection || PRINT_LC_SCORES))
    {
        std::vector<double> distances;
        distances.reserve(finalCandidates.size());
        for (const auto& lc : finalCandidates)
        {
            distances.push_back(lc.distance);
        }

        const double minSelectedDist = *std::min_element(distances.begin(), distances.end());
        const double maxSelectedDist = *std::max_element(distances.begin(), distances.end());
        const double sumDist         = std::accumulate(distances.begin(), distances.end(), 0.0);
        const double meanDist        = sumDist / static_cast<double>(distances.size());

        // Compute standard deviation
        double variance = 0.0;
        for (const auto d : distances)
        {
            const double diff = d - meanDist;
            variance += diff * diff;
        }
        const double stdDist = std::sqrt(variance / static_cast<double>(distances.size()));

        // Compute coefficient of variation (normalized measure of variance)
        const double cv = (meanDist > 0.0) ? (stdDist / meanDist) : 0.0;

        MRPT_LOG_INFO_STREAM(
            "Selected " << finalCandidates.size() << " LC candidates. "
                        << "Distance: [" << minSelectedDist << ", " << maxSelectedDist << "] m, "
                        << "mean=" << meanDist << " m, "
                        << "std=" << stdDist << " m, "
                        << "CV=" << cv);
    }

    return finalCandidates;
}

std::optional<FrameToFrameLoopClosure::LcIcpEdge> FrameToFrameLoopClosure::run_lc_icp(
    const LoopCandidate& lc, size_t threadIdx, bool profile)
{
    // Per-candidate profiling is disabled when candidates run in parallel, since
    // CTimeLogger forbids the same section name being timed from several threads.
    std::optional<mrpt::system::CTimeLoggerEntry> tle;
    if (profile)
    {
        tle.emplace(profiler_, "run_lc_icp");
    }

    ASSERT_(threadIdx < state_.perThreadState_.size());

    // Get point clouds for both frames (using LRU cache)
    auto pc_i = get_cached_pointcloud(lc.frame_i, threadIdx);
    auto pc_j = get_cached_pointcloud(lc.frame_j, threadIdx);

    if (!pc_i || !pc_j)
    {
        MRPT_LOG_WARN_STREAM(
            "Failed to generate point clouds for LC " << lc.frame_i << " <-> " << lc.frame_j);
        return std::nullopt;
    }

    // Initial guess from current graph
    const auto pose_i    = state_.get_pose(lc.frame_i);
    const auto pose_j    = state_.get_pose(lc.frame_j);
    auto       initGuess = (pose_j - pose_i).asTPose();

    auto& pts = state_.perThreadState_.at(threadIdx);

#ifdef MOLA_HAS_KISS_MATCHER
    if (params_.use_kiss_matcher && pts.kissMatcher != nullptr)
    {
        std::optional<mrpt::system::CTimeLoggerEntry> tle_km;
        if (profile)
        {
            tle_km.emplace(profiler_, "kiss_matcher_initial_guess");
        }

        auto extractEigen = [&](const mp2p_icp::metric_map_t& pc) -> std::vector<Eigen::Vector3f>
        {
            std::vector<Eigen::Vector3f> out;
            auto                         it = pc.layers.find(params_.kiss_matcher_layer);
            if (it == pc.layers.end())
            {
                return out;
            }
            const auto ptsMap = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(it->second);
            if (!ptsMap)
            {
                return out;
            }
            const auto& xs = ptsMap->getPointsBufferRef_x();
            const auto& ys = ptsMap->getPointsBufferRef_y();
            const auto& zs = ptsMap->getPointsBufferRef_z();
            out.reserve(xs.size());
            for (size_t k = 0; k < xs.size(); k++)
            {
                out.emplace_back(xs[k], ys[k], zs[k]);
            }
            return out;
        };

        const auto src_pts = extractEigen(*pc_j);
        const auto tgt_pts = extractEigen(*pc_i);

        if (!src_pts.empty() && !tgt_pts.empty())
        {
            auto*      km  = static_cast<kiss_matcher::KISSMatcher*>(pts.kissMatcher.get());
            const auto sol = km->estimate(src_pts, tgt_pts);
            // KISS-Matcher's `valid` flag only requires one surviving inlier;
            // gate on the actual final-inlier count so grossly wrong global
            // registrations (which mislead ICP) fall back to the graph guess.
            const auto nInliers = km->getNumFinalInliers();
            if (sol.valid && nInliers >= params_.kiss_matcher_min_inliers)
            {
                mrpt::math::CMatrixDouble44 T = mrpt::math::CMatrixDouble44::Identity();
                for (int r = 0; r < 3; r++)
                {
                    for (int c = 0; c < 3; c++)
                    {
                        T(r, c) = sol.rotation(r, c);
                    }
                }
                T(0, 3)   = sol.translation(0);
                T(1, 3)   = sol.translation(1);
                T(2, 3)   = sol.translation(2);
                initGuess = mrpt::poses::CPose3D(T).asTPose();
                MRPT_LOG_DEBUG_STREAM(
                    "KISS-Matcher valid guess for LC " << lc.frame_i << "<->" << lc.frame_j
                                                       << " inliers=" << nInliers
                                                       << " T=" << initGuess);
            }
            else
            {
                MRPT_LOG_DEBUG_STREAM(
                    "KISS-Matcher rejected for LC "
                    << lc.frame_i << "<->" << lc.frame_j << " (valid=" << sol.valid
                    << " inliers=" << nInliers << "); using graph-based guess");
            }
        }
    }
#endif

    // Run ICP

    update_dynamic_variables(lc.frame_j, threadIdx);

    mp2p_icp::Results icp_result;
    pts.pipeline.icp->align(*pc_j, *pc_i, initGuess, params_.icp_parameters, icp_result);

    const auto poseDelta = (icp_result.optimal_tf.getMeanVal().asTPose() - initGuess);

    MRPT_LOG_INFO_STREAM(
        "ICP " << lc.frame_i << " <-> " << lc.frame_j << " distance=" << lc.distance
               << " score=" << lc.score << " icp_quality=" << (100.0 * icp_result.quality)
               << "% iters=" << icp_result.nIterations << " Δp=" << poseDelta.translation().norm()
               << " [m] ΔR="
               << mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(poseDelta.getRotationMatrix()).norm())
               << " [deg]");

    if (icp_result.quality < params_.min_icp_goodness)
    {
        return std::nullopt;
    }

    // Edge noise: ICP covariance diagonal inflated by an additive floor. This
    // is the same recipe the graph BetweenFactor uses, so both the graph path
    // (process_loop_candidate) and the detector path (analyze) see identical
    // uncertainty.
    // covDiag is in MRPT order: x, y, z, yaw, pitch, roll.
    const auto covDiag = icp_result.optimal_tf.cov.asEigen().diagonal().array().sqrt();

    LcIcpEdge edge;
    edge.quality = icp_result.quality;

    // gtsam Pose3 tangent order: [rot_x rot_y rot_z, x y z].
    edge.sigmas << covDiag[5] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[4] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[3] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[0] + params_.icp_edge_additional_noise_xyz,
        covDiag[1] + params_.icp_edge_additional_noise_xyz,
        covDiag[2] + params_.icp_edge_additional_noise_xyz;

    // Mirror the same inflated diagonal into an MRPT covariance (order x, y, z,
    // yaw, pitch, roll) so callers that don't speak gtsam get the identical
    // edge uncertainty.
    const auto sq     = [](double v) { return v * v; };
    edge.relPose.mean = icp_result.optimal_tf.mean;
    edge.relPose.cov.setZero();
    edge.relPose.cov(0, 0) = sq(edge.sigmas[3]);  // x
    edge.relPose.cov(1, 1) = sq(edge.sigmas[4]);  // y
    edge.relPose.cov(2, 2) = sq(edge.sigmas[5]);  // z
    edge.relPose.cov(3, 3) = sq(edge.sigmas[2]);  // yaw
    edge.relPose.cov(4, 4) = sq(edge.sigmas[1]);  // pitch
    edge.relPose.cov(5, 5) = sq(edge.sigmas[0]);  // roll

    return edge;
}

std::optional<size_t> FrameToFrameLoopClosure::process_loop_candidate(const LoopCandidate& lc)
{
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "process_loop_candidate");

    // Self-optimizing path is sequential: use slot 0 and keep profiling on.
    const auto edge = run_lc_icp(lc, /*threadIdx=*/0);
    if (!edge)
    {
        return std::nullopt;
    }

    // Add ICP edge to graph
    const size_t newFactorIdx = state_.graphFG.size();
    const auto   deltaPose    = mrpt::gtsam_wrappers::toPose3(edge->relPose.mean);

    auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(edge->sigmas);

    // LC edges use plain Gaussian noise (no robust kernel here).
    // The GNC optimizer handles outlier rejection for these edges.
    state_.graphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(lc.frame_i), X(lc.frame_j), deltaPose, edgeNoise);

    accepted_lc_edges_.emplace_back(lc.frame_i, lc.frame_j);

    return newFactorIdx;
}

std::vector<ProposedLoopEdge> FrameToFrameLoopClosure::analyze(
    const mrpt::maps::CSimpleMap& snapshot, const LoopClosureAnalyzeOptions& opts)
{
    using gtsam::symbol_shorthand::X;

    const auto& sm = snapshot;

    ASSERT_(state_.initialized);

    mrpt::system::CTimeLoggerEntry tle(profiler_, "analyze");

    // Detector-only pass: we own no graph and must not mutate the map. Set up
    // just enough state for candidate search + per-candidate ICP.
    state_.sm               = &sm;
    state_.readOnlySnapshot = true;

    // Always clear the borrowed snapshot pointer and read-only flag on exit
    // (including on an early return or exception), so no dangling pointer or
    // stale flag survives into later calls.
    struct SnapshotGuard
    {
        State* st;
        ~SnapshotGuard()
        {
            st->sm               = nullptr;
            st->readOnlySnapshot = false;
        }
    } snapshotGuard{&state_};

    state_.pcCacheClear();
    accepted_lc_edges_.clear();

    MRPT_LOG_INFO_STREAM("analyze(): scanning simplemap with " << sm.size() << " frames");

    // Precompute which frames have mapping-capable observations (used by
    // find_loop_candidates to avoid lazy-loading raw frames per pair). This is a
    // read-only flow over a caller-owned snapshot, so unlike process() we must
    // not unload/modify the observations here.
    state_.frameHasMappingObs.assign(sm.size(), false);
    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto& kf               = sm.get(i);
        state_.frameHasMappingObs[i] = kf.sf && frame_has_mapping_observations(*kf.sf);
    }

    // Seed only the initial poses (no odometry/GNSS factors, no optimization):
    // candidate search and ICP initial guesses read these via State::get_pose().
    state_.graphValues.clear();
    state_.graphFG.resize(0);
    for (size_t i = 0; i < sm.size(); i++)
    {
        state_.graphValues.insert(X(i), mrpt::gtsam_wrappers::toPose3(frame_pose_in_simplemap(i)));
    }

    // Seed with pairs the caller already closed so candidate selection skips
    // them and spends its budget on as-yet-unclosed loops (drives the finalize
    // cascade toward new revisit regions each round).
    const std::set<std::pair<frame_id_t, frame_id_t>> alreadyChecked(
        opts.exclude_pairs.begin(), opts.exclude_pairs.end());
    const frame_id_t minLaterFrame = opts.first_new_keyframe.value_or(0);
    const auto       candidates    = find_loop_candidates(alreadyChecked, minLaterFrame);

    MRPT_LOG_INFO_STREAM(
        "analyze(): " << candidates.size() << " loop closure candidates"
                      << (minLaterFrame != 0 ? " (incremental)" : ""));

    std::vector<ProposedLoopEdge> out;
    out.reserve(candidates.size());
    std::atomic<bool> aborted{false};

    // Live progress reporting: total is fixed for the pass, evaluated grows as
    // ICP runs. The consumer derives a per-scan pending-queue depth from
    // (total - done). Kept as an atomic so both the sequential and parallel
    // paths update it uniformly.
    const std::size_t        candidatesTotal = candidates.size();
    std::atomic<std::size_t> evaluated{0};
    auto                     reportProgress = [&]
    {
        if (opts.on_progress)
        {
            opts.on_progress(evaluated.load(std::memory_order_relaxed), candidatesTotal);
        }
    };
    reportProgress();  // initial (0 / total), so a listener sees the queue fill

    // Loop-closure candidates are independent pairwise registrations, so their
    // ICP tests can run concurrently. Resolve the worker count: one per-thread
    // ICP slot (each with its own pipeline + KISS-Matcher instance), optionally
    // capped by num_icp_threads, and never more than the candidate count.
    //
    // `deterministic` does NOT turn off candidate parallelism, and that is a
    // measured decision rather than an oversight. Once the reduction underneath
    // (mp2p_icp's pairing list) is order-stable, evaluating candidates
    // concurrently is reproducible on its own: each candidate is an independent
    // registration on its own ICP slot, and the accepted edges are sorted below.
    // What is left needing a pin is the parallelism INSIDE a candidate, which
    // the scope handles. Serializing the candidates too would cost ~8x for no
    // determinism gained.
    const DeterministicScope detScope{params_.deterministic, this};

    size_t nThreads = 1;
    if (params_.parallel_icp_enabled)
    {
        const size_t slots = state_.perThreadState_.size();
        // Auto (0): use ~1/4 of the cores. mp2p_icp already parallelizes each ICP
        // internally with TBB, so one outer thread per core just oversubscribes
        // the shared TBB pool; the measured speedup flattens out by ~cores/4.
        //
        // cores/4 stays the auto value under `deterministic` too. Taking all the
        // slots instead was tried, on the theory that pinning the inner runtimes
        // leaves nothing to oversubscribe, and measured WORSE on KITTI-07 (47-50 s
        // against 40 s): the per-thread point-cloud cache is
        // pc_cache_max_bytes/slots, so more slots means a smaller cache each and
        // more clouds regenerated.
        nThreads = params_.num_icp_threads == 0 ? std::max<size_t>(1, slots / 4)
                                                : std::min(params_.num_icp_threads, slots);
        nThreads = std::clamp<size_t>(nThreads, 1, std::max<size_t>(1, candidates.size()));
    }

    // Evaluate one candidate on ICP slot `slot`; returns the accepted edge (if
    // any). A single degenerate candidate (e.g. a scan missing the registration
    // layer, or an ICP that fails to converge) must never abort the whole scan:
    // log and skip so the remaining candidates are still evaluated.
    auto evalCandidate = [&](const LoopCandidate& lc, size_t slot,
                             bool profile) -> std::optional<ProposedLoopEdge>
    {
        std::optional<LcIcpEdge> edge;
        try
        {
            edge = run_lc_icp(lc, slot, profile);
        }
        catch (const std::exception& e)
        {
            MRPT_LOG_WARN_STREAM(
                "Loop-closure candidate "
                << lc.frame_i << " <-> " << lc.frame_j
                << " skipped due to error: " << first_n_lines(e.what(), 2));
            return std::nullopt;
        }
        if (!edge)
        {
            return std::nullopt;
        }
        ProposedLoopEdge pe;
        pe.from          = lc.frame_i;
        pe.to            = lc.frame_j;
        pe.relative_pose = edge->relPose;
        pe.quality       = edge->quality;
        return pe;
    };

    if (nThreads <= 1)
    {
        // Sequential path (keeps per-candidate profiling).
        for (const auto& lc : candidates)
        {
            // Poll for cancellation before the expensive ICP step.
            if (opts.should_abort && opts.should_abort())
            {
                aborted = true;
                break;
            }
            auto pe = evalCandidate(lc, /*slot=*/0, /*profile=*/true);
            evaluated.fetch_add(1, std::memory_order_relaxed);
            reportProgress();
            if (!pe)
            {
                continue;
            }
            // Stream the edge to the consumer early, before the scan finishes.
            if (opts.on_edge_found)
            {
                opts.on_edge_found(*pe);
            }
            out.push_back(*pe);
        }
    }
    else
    {
        // Parallel path: each worker owns one ICP slot and pulls candidates from
        // a shared atomic index. Edge streaming/collection is serialized under a
        // mutex; the accepted-edge ORDER is therefore not deterministic, which
        // the robust (GNC/Huber) graph downstream tolerates. Per-candidate
        // profiling is off (CTimeLogger forbids one section across threads).
        std::atomic<size_t> nextIdx{0};
        std::mutex          outMtx;
        auto                worker = [&](size_t slot)
        {
            while (!aborted.load(std::memory_order_relaxed))
            {
                if (opts.should_abort && opts.should_abort())
                {
                    aborted = true;
                    break;
                }
                const size_t idx = nextIdx.fetch_add(1, std::memory_order_relaxed);
                if (idx >= candidates.size())
                {
                    break;
                }
                auto pe = evalCandidate(candidates[idx], slot, /*profile=*/false);
                evaluated.fetch_add(1, std::memory_order_relaxed);
                // Only serialize when there is something to do under the lock:
                // progress to report or an accepted edge to push. Rejected
                // candidates (the common case) stay on the lock-free path.
                if (opts.on_progress || pe)
                {
                    std::lock_guard<std::mutex> lk(outMtx);
                    reportProgress();
                    if (pe)
                    {
                        if (opts.on_edge_found)
                        {
                            opts.on_edge_found(*pe);
                        }
                        out.push_back(*pe);
                    }
                }
            }
        };

        // Silence the profiler across the parallel region: several helpers
        // (point-cloud generation, filter pipeline) time shared sections that
        // CTimeLogger does not allow to be entered from multiple threads.
        const bool profWasEnabled = profiler_.isEnabled();
        profiler_.enable(false);

        std::vector<std::future<void>> futs;
        futs.reserve(nThreads);
        for (size_t k = 0; k < nThreads; k++)
        {
            futs.emplace_back(threads_.enqueue(worker, k));
        }
        for (auto& f : futs)
        {
            f.get();
        }

        profiler_.enable(profWasEnabled);
    }

    // A canonical order on the way out. The sequential path already produces
    // candidate order, but a consumer folding these into a factor graph often
    // drops a pair it has already closed, which makes the order observable --
    // so state it here rather than leaving each consumer to sort defensively.
    if (params_.deterministic)
    {
        std::sort(
            out.begin(), out.end(),
            [](const ProposedLoopEdge& a, const ProposedLoopEdge& b)
            { return std::minmax(a.from, a.to) < std::minmax(b.from, b.to); });
    }

    MRPT_LOG_INFO_STREAM(
        "analyze(): accepted " << out.size() << " loop closure edges"
                               << (aborted ? " (aborted early)" : "")
                               << (params_.deterministic ? " [deterministic]" : ""));

    if (opts.out_stats)
    {
        opts.out_stats->candidates_generated = candidatesTotal;
        opts.out_stats->candidates_evaluated = evaluated.load(std::memory_order_relaxed);
        opts.out_stats->edges_accepted       = out.size();
        opts.out_stats->aborted              = aborted.load();
    }

    // state_.sm is cleared by snapshotGuard on scope exit.
    return out;
}

mp2p_icp::metric_map_t::Ptr FrameToFrameLoopClosure::generate_frame_pointcloud(
    frame_id_t frameId, size_t threadIdx)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "generate_frame_pointcloud");

    ASSERT_(state_.sm);
    const auto& [pose, sf, twist] = state_.sm->get(frameId);

    if (!frame_has_mapping_observations(*sf))
    {
        return {};
    }

    auto& pts         = state_.perThreadState_.at(threadIdx);
    auto  observation = mp2p_icp::metric_map_t::Create();

    // First, search for velocity buffer data:
    for (const auto& obs : *sf)
    {
        ASSERT_(obs);
        mp2p_icp::update_velocity_buffer_from_obs(
            pts.pipeline.parameter_source.localVelocityBuffer, obs);
    }

    update_dynamic_variables(frameId, threadIdx);

    // Next, do the actual sensor data processing:

    try
    {
        // Generate point cloud from observations
        for (const auto& obs : *sf)
        {
            mp2p_icp_filters::apply_generators(pts.pipeline.obs_generators, *obs, *observation);
        }
    }
    catch (const std::exception& e)
    {
        // If the exception msg contains "Assert file existence failed", it's due to missing
        // external files. Emit a warning and return an empty cloud for this frame,
        // but continue with the rest without quitting.
        const std::string errMsg = e.what();
        if (errMsg.find("Assert file existence failed") != std::string::npos)
        {
            MRPT_LOG_WARN_STREAM(
                "Frame " << frameId << ": Skipping observation due to missing external files: "
                         << first_n_lines(errMsg, 3));
            return {};
        }
        throw;  // Rethrow other exceptions
    }

    // Apply filters
    mp2p_icp_filters::apply_filter_pipeline(pts.pipeline.pc_filter, *observation, profiler_);

    // Some keyframes filter down to an empty cloud (e.g. a near-empty or
    // heavily-occluded scan, or a dataset whose extremes fall outside the
    // range/bounding-box filters): the ICP registration layers then end up
    // missing or empty. Treat such a frame as unusable and return an empty
    // result so the caller skips this loop-closure candidate, instead of
    // aborting the whole background scan when align() later fails to find its
    // input layers.
    if (observation->size_points_only() == 0)
    {
        MRPT_LOG_WARN_STREAM(
            "Frame " << frameId
                     << ": generated an empty point cloud; skipping it as a loop-closure "
                        "candidate.");
        return {};
    }

    // Unload raw observation data to free RAM (only effective for externally-stored data).
    // Skipped in the read-only analyze() flow, which must not mutate the snapshot.
    if (params_.unload_observations_after_use && !state_.readOnlySnapshot)
    {
        for (const auto& obs : *sf)
        {
            obs->unload();
        }
    }

    // Save local map ID, useful if generating debug ICP log files is enabled:
    observation->id = std::optional<uint64_t>(static_cast<uint64_t>(frameId));

    return observation;
}

mp2p_icp::metric_map_t::Ptr FrameToFrameLoopClosure::get_cached_pointcloud(
    frame_id_t frameId, size_t threadIdx)
{
    // Cache disabled?
    if (params_.pc_cache_max_bytes == 0)
    {
        return generate_frame_pointcloud(frameId, threadIdx);
    }

    // Each thread owns its own cache slot, so no locking is needed and two
    // candidates never share a cloud (which would race its lazy KD-tree).
    auto& pts = state_.perThreadState_.at(threadIdx);

    // Cache hit?
    auto it = pts.pcCache.find(frameId);
    if (it != pts.pcCache.end())
    {
        pts.pcLruOrder.remove(frameId);  // move to front of LRU list
        pts.pcLruOrder.push_front(frameId);
        return it->second.pc;
    }

    // Cache miss: generate the point cloud.
    auto pc = generate_frame_pointcloud(frameId, threadIdx);
    if (!pc)
    {
        return {};
    }

    // Estimate memory usage (sum of all point cloud layer sizes)
    size_t approxBytes = 0;
    for (const auto& [layerName, map] : pc->layers)
    {
        if (map)
        {
            // Use the number of points * approximate bytes per point
            auto ptsMap = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(map);
            if (ptsMap)
            {
                approxBytes += ptsMap->size() * (3 * sizeof(float) + 16);  // xyz + overhead
            }
        }
    }
    if (approxBytes == 0)
    {
        approxBytes = 1024;  // minimum estimate
    }

    // Insert into this thread's cache.
    pts.pcCache[frameId] = {pc, approxBytes};
    pts.pcLruOrder.push_front(frameId);
    pts.pcCacheTotalBytes += approxBytes;
    evict_pc_cache(pts);

    return pc;
}

void FrameToFrameLoopClosure::evict_pc_cache(PerThreadState& pts)
{
    // Budget is divided across the ICP slots so the total cache footprint stays
    // close to pc_cache_max_bytes regardless of the worker-thread count.
    const size_t budget =
        std::max<size_t>(1, params_.pc_cache_max_bytes / state_.perThreadState_.size());
    while (pts.pcCacheTotalBytes > budget && !pts.pcLruOrder.empty())
    {
        const auto oldestId = pts.pcLruOrder.back();
        pts.pcLruOrder.pop_back();

        auto it = pts.pcCache.find(oldestId);
        if (it != pts.pcCache.end())
        {
            pts.pcCacheTotalBytes -= it->second.approxBytes;
            pts.pcCache.erase(it);
        }
    }
}

FrameToFrameLoopClosure::OptGraphResult FrameToFrameLoopClosure::optimize_graph()
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "optimize_graph");

    ASSERT_(!state_.graphFG.empty());

    MRPT_LOG_INFO_STREAM("Executing GNC optimization...");

    const auto result = lc_common::run_gnc(
        state_.graphFG, state_.planarityFG, state_.graphValues, state_.knownInlierFactorIndices,
        this);

    MRPT_LOG_INFO_STREAM(
        "GNC result: " << result.numLcInliers << " LC inlier(s), " << result.numLcOutliers
                       << " LC outlier(s) rejected");

    state_.graphValues = result.values;

    // Recompute combined FG for marginals (same as in run_gnc)
    gtsam::NonlinearFactorGraph combined = state_.graphFG;
    if (!state_.planarityFG.empty())
    {
        combined.add(state_.planarityFG);
    }
    try
    {
        state_.graphMarginals.emplace(combined, state_.graphValues);
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_WARN_STREAM("Could not compute graph marginals: " << e.what());
    }

    auto bckCol =
        mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO);
    mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO) =
        mrpt::system::ConsoleForegroundColor::BRIGHT_GREEN;
    MRPT_LOG_INFO_STREAM(
        "Graph optimized (GNC): RMSE " << result.rmseInit << " -> " << result.rmseEnd
                                       << ", largest delta: " << result.largestDelta << " m");
    mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO) = bckCol;

    return {result.largestDelta, result.numLcInliers, result.numLcOutliers};
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

mrpt::math::CMatrixDouble66 FrameToFrameLoopClosure::State::get_pose_cov(frame_id_t id) const
{
    using gtsam::symbol_shorthand::X;
    ASSERT_(graphMarginals.has_value());

    return mrpt::gtsam_wrappers::to_mrpt_se3_cov6(graphMarginals->marginalCovariance(X(id)));
}

void FrameToFrameLoopClosure::update_dynamic_variables(frame_id_t frameId, size_t threadIdx)
{
    auto& pts = state_.perThreadState_.at(threadIdx);
    auto& ps  = pts.pipeline.parameter_source;

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

    if (!pts.pipeline.expr_threshold_sigma_final.is_compiled())
    {
        pts.pipeline.expr_threshold_sigma_final.compile(
            params_.threshold_sigma_final, {}, "expr_threshold_sigma_final");

        pts.pipeline.expr_threshold_sigma_initial.compile(
            params_.threshold_sigma_initial, {}, "expr_threshold_sigma_initial");
    }

    ps.updateVariable("SIGMA_INIT", pts.pipeline.expr_threshold_sigma_initial.eval());
    ps.updateVariable("SIGMA_FINAL", pts.pipeline.expr_threshold_sigma_final.eval());
    ps.updateVariable("ESTIMATED_SENSOR_MAX_RANGE", params_.max_sensor_range);

    // This will be overwritten by the actual ICP loop later on,
    // but we need to define all variables before building a local map:
    ps.updateVariable("ICP_ITERATION", 0);

    ps.realize();
}

void FrameToFrameLoopClosure::save_3d_scene_initial_files() const
{
    ASSERT_(state_.sm);
    const auto& sm     = *state_.sm;
    const auto& prefix = params_.debug_files_prefix;

    const auto pathColor = mrpt::img::TColorf(
                               params_.scene_path_color_r, params_.scene_path_color_g,
                               params_.scene_path_color_b, params_.scene_path_color_a)
                               .asTColor();

    // 1) Initial path edges
    {
        auto lines = mrpt::viz::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_path_line_width);
        lines->setColor_u8(pathColor);

        for (size_t i = 1; i < sm.size(); i++)
        {
            const auto p0 = frame_pose_in_simplemap(i - 1).translation();
            const auto p1 = frame_pose_in_simplemap(i).translation();
            lines->appendLine(p0, p1);
        }

        mrpt::viz::Scene scene;
        scene.insert(lines);
        const auto fn = prefix + "initial_path_edges.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }

    // 2) Initial keyframe points
    {
        auto pts = mrpt::viz::CPointCloud::Create();
        pts->setPointSize(params_.scene_keyframe_point_size);
        pts->setColor_u8(pathColor);

        for (size_t i = 0; i < sm.size(); i++)
        {
            const auto p = frame_pose_in_simplemap(i).translation();
            pts->insertPoint(p);
        }

        mrpt::viz::Scene scene;
        scene.insert(pts);
        const auto fn = prefix + "initial_keyframe_points.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }
}

void FrameToFrameLoopClosure::save_3d_scene_files(const std::string& suffix) const
{
    ASSERT_(state_.sm);
    const auto& sm     = *state_.sm;
    const auto  prefix = params_.debug_files_prefix + (suffix.empty() ? "" : suffix + "_");

    // 1) Path edges: lines connecting consecutive keyframes
    {
        auto lines = mrpt::viz::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_path_line_width);
        lines->setColor_u8(mrpt::img::TColorf(
                               params_.scene_path_color_r, params_.scene_path_color_g,
                               params_.scene_path_color_b, params_.scene_path_color_a)
                               .asTColor());

        for (size_t i = 1; i < sm.size(); i++)
        {
            const auto p0 = state_.get_pose(i - 1).translation();
            const auto p1 = state_.get_pose(i).translation();
            lines->appendLine(p0, p1);
        }

        mrpt::viz::Scene scene;
        scene.insert(lines);
        const auto fn = prefix + "path_edges.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }

    // 2) Keyframe points
    {
        auto pts = mrpt::viz::CPointCloud::Create();
        pts->setPointSize(params_.scene_keyframe_point_size);
        pts->setColor_u8(mrpt::img::TColorf(
                             params_.scene_path_color_r, params_.scene_path_color_g,
                             params_.scene_path_color_b, params_.scene_path_color_a)
                             .asTColor());

        for (size_t i = 0; i < sm.size(); i++)
        {
            const auto p = state_.get_pose(i).translation();
            pts->insertPoint(p);
        }

        mrpt::viz::Scene scene;
        scene.insert(pts);
        const auto fn = prefix + "keyframe_points.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }

    // 3) Loop closure edges
    {
        auto lines = mrpt::viz::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_lc_line_width);
        lines->setColor_u8(mrpt::img::TColorf(
                               params_.scene_lc_color_r, params_.scene_lc_color_g,
                               params_.scene_lc_color_b, params_.scene_lc_color_a)
                               .asTColor());

        for (const auto& [fi, fj] : accepted_lc_edges_)
        {
            const auto p0 = state_.get_pose(fi).translation();
            const auto p1 = state_.get_pose(fj).translation();
            lines->appendLine(p0, p1);
        }

        mrpt::viz::Scene scene;
        scene.insert(lines);
        const auto fn = prefix + "lc_edges.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }
}

void FrameToFrameLoopClosure::build_planarity_factors(double sigmaZ, double sigmaAng)
{
    ASSERT_(state_.sm);
    lc_common::build_planarity_factors(
        state_.planarityFG, state_.graphValues, state_.sm->size(), sigmaZ, sigmaAng);
}

void FrameToFrameLoopClosure::save_3d_scene_live_preview(
    const std::vector<LoopCandidate>& pendingCandidates, const LivePreviewStats& stats) const
{
    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    mrpt::viz::Scene scene;

    // Ground grid spanning the trajectory bounding box
    {
        constexpr float GRID_SPACING = 5.0f;
        constexpr float MARGIN       = 10.0f;

        float xMin = std::numeric_limits<float>::max();
        float xMax = -std::numeric_limits<float>::max();
        float yMin = std::numeric_limits<float>::max();
        float yMax = -std::numeric_limits<float>::max();

        for (size_t i = 0; i < sm.size(); i++)
        {
            const auto p = state_.get_pose(i).translation();
            xMin         = std::min(xMin, static_cast<float>(p.x));
            xMax         = std::max(xMax, static_cast<float>(p.x));
            yMin         = std::min(yMin, static_cast<float>(p.y));
            yMax         = std::max(yMax, static_cast<float>(p.y));
        }

        // Snap outward to the nearest grid line
        xMin = std::floor((xMin - MARGIN) / GRID_SPACING) * GRID_SPACING;
        xMax = std::ceil((xMax + MARGIN) / GRID_SPACING) * GRID_SPACING;
        yMin = std::floor((yMin - MARGIN) / GRID_SPACING) * GRID_SPACING;
        yMax = std::ceil((yMax + MARGIN) / GRID_SPACING) * GRID_SPACING;

        auto grid = mrpt::viz::CGridPlaneXY::Create(xMin, xMax, yMin, yMax, 0.0f, GRID_SPACING);
        grid->setColor(0.5f, 0.5f, 0.5f, 0.5f);
        scene.insert(grid);
    }

    // Trajectory path edges
    {
        auto lines = mrpt::viz::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_path_line_width);
        lines->setColor_u8(mrpt::img::TColorf(
                               params_.scene_path_color_r, params_.scene_path_color_g,
                               params_.scene_path_color_b, params_.scene_path_color_a)
                               .asTColor());
        for (size_t i = 1; i < sm.size(); i++)
        {
            lines->appendLine(
                state_.get_pose(i - 1).translation(), state_.get_pose(i).translation());
        }
        scene.insert(lines);
    }

    // Keyframe positions
    {
        auto pts = mrpt::viz::CPointCloud::Create();
        pts->setPointSize(params_.scene_keyframe_point_size);
        pts->setColor_u8(mrpt::img::TColorf(
                             params_.scene_path_color_r, params_.scene_path_color_g,
                             params_.scene_path_color_b, params_.scene_path_color_a)
                             .asTColor());
        for (size_t i = 0; i < sm.size(); i++)
        {
            pts->insertPoint(state_.get_pose(i).translation());
        }
        scene.insert(pts);
    }

    // Accepted LC edges (green)
    if (!accepted_lc_edges_.empty())
    {
        auto lines = mrpt::viz::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_lc_line_width);
        lines->setColor_u8(mrpt::img::TColorf(
                               params_.scene_lc_color_r, params_.scene_lc_color_g,
                               params_.scene_lc_color_b, params_.scene_lc_color_a)
                               .asTColor());
        for (const auto& [fi, fj] : accepted_lc_edges_)
        {
            lines->appendLine(state_.get_pose(fi).translation(), state_.get_pose(fj).translation());
        }
        scene.insert(lines);
    }

    // Pending candidate LC edges (orange)
    if (!pendingCandidates.empty())
    {
        auto lines = mrpt::viz::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_lc_line_width);
        lines->setColor_u8(
            mrpt::img::TColorf(
                params_.scene_lc_candidate_color_r, params_.scene_lc_candidate_color_g,
                params_.scene_lc_candidate_color_b, params_.scene_lc_candidate_color_a)
                .asTColor());
        for (const auto& lc : pendingCandidates)
        {
            lines->appendLine(
                state_.get_pose(lc.frame_i).translation(),
                state_.get_pose(lc.frame_j).translation());
        }
        scene.insert(lines);
    }

    // GPS/GNSS readings as ENU point cloud (cyan, size 3, alpha 50%)
    size_t gnssPointCount = 0;
    if (params_.use_gnss)
    {
        const auto gnssFrames = extract_gnss_frames_from_sm(*state_.sm, state_.globalGeoRef);
        if (!gnssFrames.frames.empty())
        {
            auto pts = mrpt::viz::CPointCloud::Create();
            pts->setPointSize(3.0f);
            pts->setColor_u8(mrpt::img::TColor(0, 220, 220, 128));  // cyan, alpha=50%
            for (const auto& gf : gnssFrames.frames)
            {
                pts->insertPoint(gf.enu);
            }
            gnssPointCount = gnssFrames.frames.size();
            scene.insert(pts);
        }
    }

    // Text overlay on the main viewport
    {
        auto vp = scene.getViewport("main");

        mrpt::viz::TFontParams fp;
        fp.vfont_name  = "sans";
        fp.vfont_scale = 14.0f;
        fp.draw_shadow = true;

        fp.color = mrpt::img::TColorf(1.0f, 1.0f, 1.0f);
        vp->addTextMessage(0.02, -20.0, "FrameToFrameLoopClosure - live preview", 0, fp);

        fp.color = mrpt::img::TColorf(0.9f, 0.9f, 0.3f);
        vp->addTextMessage(
            0.02, -42.0, mrpt::format("LC round: %zu / %zu", stats.lcRound + 1, stats.totalRounds),
            1, fp);

        fp.color = mrpt::img::TColorf(
            params_.scene_lc_candidate_color_r, params_.scene_lc_candidate_color_g,
            params_.scene_lc_candidate_color_b);
        vp->addTextMessage(
            0.02, -64.0,
            mrpt::format(
                "Candidates evaluated: %zu / %zu  (pending: %zu)", stats.candidatesDone,
                stats.candidatesTotal, pendingCandidates.size()),
            2, fp);

        fp.color = mrpt::img::TColorf(
            params_.scene_lc_color_r, params_.scene_lc_color_g, params_.scene_lc_color_b);
        vp->addTextMessage(
            0.02, -86.0,
            mrpt::format(
                "Accepted loop closures: %zu (GNC: %zu inliers, %zu outliers rejected)",
                stats.acceptedLCs, stats.gncInliers, stats.gncOutliers),
            3, fp);

        fp.color = mrpt::img::TColorf(0.7f, 0.7f, 0.7f);
        vp->addTextMessage(0.02, -108.0, mrpt::format("Keyframes: %zu", sm.size()), 4, fp);

        if (params_.use_gnss)
        {
            fp.color = mrpt::img::TColorf(0.0f, 0.86f, 0.86f);  // cyan
            vp->addTextMessage(
                0.02, -130.0, mrpt::format("GPS/GNSS readings (ENU): %zu", gnssPointCount), 5, fp);
        }
    }

    const auto fn    = params_.debug_files_prefix + "live_preview.3Dscene";
    const auto tmpFn = fn + ".tmp";
    if (!scene.saveToFile(tmpFn))
    {
        MRPT_LOG_WARN_STREAM("Failed to save live preview 3D scene: " << tmpFn);
        return;
    }

    if (::rename(tmpFn.c_str(), fn.c_str()) != 0)
    {
        MRPT_LOG_WARN_STREAM(
            "Failed to atomically publish live preview scene: " << fn
                                                                << " error=" << strerror(errno));
    }
}

void FrameToFrameLoopClosure::save_trajectory_as_tum(
    const std::string& filename, bool saveCovariancesToo) const
{
    ASSERT_(state_.sm);
    if (saveCovariancesToo)
    {
        ASSERT_(state_.graphMarginals.has_value());
    }

    std::function<mrpt::math::CMatrixDouble66(size_t)> covOf;
    if (saveCovariancesToo)
    {
        covOf = [this](size_t id) { return state_.get_pose_cov(id); };
    }

    lc_common::save_trajectory_as_tum(
        filename, *state_.sm, [this](size_t id) { return state_.get_pose(id); },
        saveCovariancesToo ? &covOf : nullptr, const_cast<FrameToFrameLoopClosure*>(this));
}