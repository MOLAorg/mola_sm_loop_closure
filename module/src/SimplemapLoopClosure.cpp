/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

// MRPT:
#include <mola_sm_loop_closure/simplemap_georeference.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CEllipsoid2D.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>

// MOLA:
#include <mola_relocalization/relocalization.h>
#include <mola_sm_loop_closure/SimplemapLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>

// MRPT graphs:
#include <mrpt/graphs/dijkstra.h>

// visualization:
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/graph_tools.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

using namespace mola;

const bool PRINT_ALL_SCORES = mrpt::get_env<bool>("PRINT_ALL_SCORES", false);
const bool SAVE_LCS         = mrpt::get_env<bool>("SAVE_LCS", false);
const bool SAVE_TREES       = mrpt::get_env<bool>("SAVE_TREES", false);

namespace
{
mrpt::math::TBoundingBox SimpleMapBoundingBox(const mrpt::maps::CSimpleMap& sm)
{
    // estimate path bounding box:
    auto bbox = mrpt::math::TBoundingBox::PlusMinusInfinity();

    for (const auto& [pose, sf, twist] : sm)
    {
        const auto p = pose->getMeanVal().asTPose();
        bbox.updateWithPoint(p.translation());
    }

    return bbox;
}

}  // namespace

SimplemapLoopClosure::SimplemapLoopClosure()
{
    mrpt::system::COutputLogger::setLoggerName("SimplemapLoopClosure");
    threads_.name("sm_localmaps_build");
}

SimplemapLoopClosure::~SimplemapLoopClosure() = default;

void SimplemapLoopClosure::initialize(const mrpt::containers::yaml& c)
{
    MRPT_TRY_START

    // Load params:
    const auto cfg = c["params"];

    if (cfg["lidar_sensor_labels"].isSequence())
    {
        const auto seq = cfg["lidar_sensor_labels"].asSequence();
        for (const auto& sl : seq)
        {
            const auto s = sl.as<std::string>();
            MRPT_LOG_DEBUG_STREAM("Adding as input lidar sensor label: " << s);
            params_.lidar_sensor_labels.emplace_back(s);
        }
    }
    else
    {
        ASSERT_(cfg["lidar_sensor_labels"].isScalar());
        const auto s = cfg["lidar_sensor_labels"].as<std::string>();
        MRPT_LOG_DEBUG_STREAM("Adding as input lidar sensor label: " << s);
        params_.lidar_sensor_labels.emplace_back(s);
    }
    ASSERT_(!params_.lidar_sensor_labels.empty());

    if (cfg.has("gnns_sensor_label"))
        params_.gnns_sensor_label = cfg["gnns_sensor_label"].as<std::string>();

    YAML_LOAD_REQ(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, profiler_enabled, bool);
    YAML_LOAD_REQ(params_, submap_max_length_wrt_map, double);
    YAML_LOAD_OPT(params_, do_first_gross_relocalize, bool);
    YAML_LOAD_OPT(params_, do_montecarlo_icp, bool);
    YAML_LOAD_OPT(params_, assume_planar_world, bool);
    YAML_LOAD_OPT(params_, use_gnns, bool);

    YAML_LOAD_REQ(params_, threshold_sigma_initial, double);
    YAML_LOAD_REQ(params_, threshold_sigma_final, double);
    YAML_LOAD_REQ(params_, max_sensor_range, double);

    YAML_LOAD_REQ(params_, icp_edge_robust_param, double);
    YAML_LOAD_REQ(params_, icp_edge_worst_multiplier, double);
    YAML_LOAD_OPT(params_, max_number_lc_candidates, uint32_t);

    YAML_LOAD_OPT(
        params_, min_volume_intersection_ratio_for_lc_candidate, double);

    YAML_LOAD_OPT(params_, save_submaps_viz_files, bool);

    // system-wide profiler:
    profiler_.enable(params_.profiler_enabled);

    ENSURE_YAML_ENTRY_EXISTS(c, "icp_settings");
    ASSERT_(c["insert_observation_into_local_map"].isSequence());

    mrpt::system::CTimeLoggerEntry tlePcInit(
        profiler_, "filterPointCloud_initialize");

    for (size_t threadIdx = 0; threadIdx < state_.perThreadState_.size();
         threadIdx++)
    {
        auto& pts = state_.perThreadState_.at(threadIdx);

        const auto [icp, icpParams] =
            mp2p_icp::icp_pipeline_from_yaml(c["icp_settings"]);

        pts.icp                = icp;
        params_.icp_parameters = icpParams;

        // Attach all ICP instances to the parameter source for dynamic
        // parameters:
        pts.icp->attachToParameterSource(pts.parameter_source);

        // Obs2map merge pipeline:
        pts.obs2map_merge = mp2p_icp_filters::filter_pipeline_from_yaml(
            c["insert_observation_into_local_map"]);

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            pts.obs2map_merge, pts.parameter_source);

        ASSERT_(!pts.obs2map_merge.empty());

        // Create lidar segmentation algorithm:
        // Observation -> map generator:
        if (c.has("observations_generator") &&
            !c["observations_generator"].isNullNode())
        {
            pts.obs_generators = mp2p_icp_filters::generators_from_yaml(
                c["observations_generator"]);
        }
        else
        {
            std::cout
                << "[warning] Using default mp2p_icp_filters::Generator for "
                   "observations since no YAML 'observations_generator' entry "
                   "was given\n";

            auto defaultGen = mp2p_icp_filters::Generator::Create();
            defaultGen->initialize({});
            pts.obs_generators.push_back(defaultGen);
        }

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            pts.obs_generators, pts.parameter_source);

        if (c.has("observations_filter"))
        {
            pts.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(
                c["observations_filter"]);

            // Attach to the parameter source for dynamic parameters:
            mp2p_icp::AttachToParameterSource(
                pts.pc_filter, pts.parameter_source);
        }

        // Local map generator:
        if (c.has("localmap_generator") &&
            !c["localmap_generator"].isNullNode())
        {
            pts.local_map_generators =
                mp2p_icp_filters::generators_from_yaml(c["localmap_generator"]);
        }
        else
        {
            ASSERT_(
                "Providing a 'localmap_generator' is mandatory in this "
                "application.");
        }
        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            pts.local_map_generators, pts.parameter_source);

        // submaps final stage filter:
        pts.submap_final_filter = mp2p_icp_filters::filter_pipeline_from_yaml(
            c["submap_final_filter"]);

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            pts.submap_final_filter, pts.parameter_source);

    }  // end for threadIdx
    tlePcInit.stop();

    state_.initialized = true;

    MRPT_TRY_END
}

namespace
{
bool sf_has_real_mapping_observations(const mrpt::obs::CSensoryFrame& sf)
{
    if (sf.empty()) return false;
    if (auto oPC =
            sf.getObservationByClass<mrpt::obs::CObservationPointCloud>();
        oPC)
        return true;

    if (auto o2D =
            sf.getObservationByClass<mrpt::obs::CObservation2DRangeScan>();
        o2D)
        return true;

    if (auto o3D =
            sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>();
        o3D)
        return true;

    if (auto oVl =
            sf.getObservationByClass<mrpt::obs::CObservationVelodyneScan>();
        oVl)
        return true;

    // We don't recognize any valid mapping-suitable observation in the SF.
    return false;
}

}  // namespace

// Find and apply loop closures in the input/output simplemap
void SimplemapLoopClosure::process(mrpt::maps::CSimpleMap& sm)
{
    using namespace std::string_literals;

    ASSERT_(state_.initialized);

    state_.sm = &sm;

    // Minimum to have a large-enough topological loop closure:
    // ASSERT_GT_(sm.size(), 3 * params_.submap_keyframe_count);

    // Build submaps:
    std::vector<std::set<keyframe_id_t>> detectedSubMaps;

    {
        std::set<keyframe_id_t> pendingKFs;
        bool                    anyValidObsInPendingSet = false;

        const auto   bbox     = SimpleMapBoundingBox(sm);
        const double smLength = (bbox.max - bbox.min).norm();

        const double max_submap_length =
            params_.submap_max_length_wrt_map * smLength;

        MRPT_LOG_INFO_FMT("Using submap length=%.02f m", max_submap_length);

        for (size_t i = 0; i < sm.size(); i++)
        {
            pendingKFs.insert(i);

            const auto pose_i_local =
                keyframe_relative_pose_in_simplemap(i, *pendingKFs.begin());

            const auto& [pose_i, sf_i, twist_i] = state_.sm->get(i);

            // don't cut a submap while we are processing empty SFs since we
            // don't know for how long it will take and we might end up with a
            // totally empty final submap
            if (!sf_has_real_mapping_observations(*sf_i)) continue;
            anyValidObsInPendingSet = true;

            if (pose_i_local.translation().norm() >= max_submap_length)
            {
                detectedSubMaps.emplace_back(pendingKFs);
                pendingKFs.clear();
                anyValidObsInPendingSet = false;
            }
        }
        // remaining ones?
        if (!pendingKFs.empty())
        {
            if (anyValidObsInPendingSet)
            {
                detectedSubMaps.emplace_back(pendingKFs);
            }
            else
            {
                // just append to the last submap, since none of the SFs has
                // data to build a new local map
                for (const auto id : pendingKFs)
                    detectedSubMaps.back().insert(id);
            }
        }
    }

    // process pending submap creation, in parallel threads:
    const size_t nSubMaps = detectedSubMaps.size();

    for (submap_id_t submapId = 0; submapId < nSubMaps; submapId++)
    {
        // Modify the submaps[] std::map here in this main thread:
        SubMap& submap = state_.submaps[submapId];
        submap.id      = submapId;

        build_submap_from_kfs_into(detectedSubMaps.at(submapId), submap);

        MRPT_LOG_INFO_STREAM(
            "Done with submap #" << submapId << " / " << nSubMaps);
    }

    // Build a graph with the submaps:
    // -----------------------------------------------
    // Nodes:
    for (const auto& [id, submap] : state_.submaps)
    {
        // These are poses without uncertainty:
        auto& globalPose = state_.submapsGraph.nodes[id];
        globalPose       = submap.global_pose;
    }
    // Edges: one between adjacent submaps in traverse order:
    {
        std::optional<const SubMap*> lastSubmap;
        for (const auto& [id, submap] : state_.submaps)
        {
            if (lastSubmap)
            {
                // add edge: i-1 => i
                const auto last_id = lastSubmap.value()->id;
                const auto this_id = id;

                const auto& thisGlobalPose = state_.submapsGraph.nodes[this_id];
                const auto& lastGlobalPose = state_.submapsGraph.nodes[last_id];

                mrpt::poses::CPose3DPDFGaussian relPose;
                relPose.mean = thisGlobalPose - lastGlobalPose;

                // Note: this uncertainty for the SUBMAPS graph is used to
                // estimate submap overlap probabilities:
                using namespace mrpt::literals;  // _deg

                relPose.cov.setDiagonal(
                    {mrpt::square(0.10), mrpt::square(0.10), mrpt::square(0.15),
                     mrpt::square(1.0_deg), mrpt::square(1.5_deg),
                     mrpt::square(1.0_deg)});

                state_.submapsGraph.insertEdge(last_id, this_id, relPose);
            }

            lastSubmap = &submap;
        }
    }

    // Build a graph with the low-level keyframes:
    // -----------------------------------------------
    using gtsam::symbol_shorthand::X;  // poses SE(3)

    for (const auto& [submapId, submap] : state_.submaps)
    {
        for (const auto id : submap.kf_ids)
        {
            const auto         pose_i = keyframe_pose_in_simplemap(id);
            const gtsam::Pose3 p      = mrpt::gtsam_wrappers::toPose3(pose_i);

            state_.kfGraphValues.insert(X(id), p);

            // anchor for first KF:
            if (state_.kfGraphFG.empty())
            {
                state_.kfGraphFG
                    .emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(id), p);
                state_.kfGraphFGRobust
                    .emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(id), p);
            }
        }
    }

    // create edges: i -> i-1
    for (size_t i = 1; i < sm.size(); i++)
    {
        // Extract cov from simplemap cov from icp odometry:
        mrpt::poses::CPose3DPDFGaussian ppi, ppim1;
        {
            const auto& [pose_i, sf_i, twist_i]       = state_.sm->get(i);
            const auto& [pose_im1, sf_im1, twist_im1] = state_.sm->get(i - 1);

            ppi.copyFrom(*pose_i);
            ppim1.copyFrom(*pose_im1);
        }

        const mrpt::poses::CPose3DPDFGaussian relPose = ppi - ppim1;

        const gtsam::Vector6 sigmasXYZYPR =
            relPose.cov.asEigen().diagonal().array().sqrt().eval();

#if 0
        MRPT_LOG_INFO_STREAM(
            "[FG] Adding edge: "
            << i - 1 << " => " << i << " pose: " << relPose.getPoseMean()
            << " sigmas: " << sigmasXYZYPR.transpose() << "\n"
            << "relPose: " << relPose.cov << "\n"
            << "ppi: " << ppi << "\n"
            << "ppim1: " << ppim1 << "\n\n");
#endif

        const gtsam::Pose3 deltaPose =
            mrpt::gtsam_wrappers::toPose3(relPose.getPoseMean());

        gtsam::Vector6 sigmas;
        sigmas << sigmasXYZYPR[5], sigmasXYZYPR[4], sigmasXYZYPR[3],  //
            sigmasXYZYPR[0], sigmasXYZYPR[1], sigmasXYZYPR[2];

        auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

        auto f = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);

        state_.kfGraphFG += f;
        state_.kfGraphFGRobust += f;
    }

    // GNNS Edges: additional edges in both graphs:
    if (state_.globalGeoRef.has_value())
    {
        const auto  ref_id    = state_.globalGeoRefSubmapId;
        const auto& refSubmap = state_.submaps.at(ref_id);

        for (const auto& [id, submap] : state_.submaps)
        {
            // has this submap GNNS?
            if (!submap.geo_ref) continue;

            // add edge: gpsRefId => id
            const auto this_id = id;

            // T_0_i = (T_enu_0)⁻¹ · T_enu_i (notes picture! pass to paper)
            const auto T_enu_0 = refSubmap.geo_ref->T_enu_to_map;
            auto       T_0_enu = -T_enu_0;
            T_0_enu.cov.setZero();  // Ignore uncertainty of this first T

            const auto T_enu_i = submap.geo_ref->T_enu_to_map;

            const mrpt::poses::CPose3DPDFGaussian relPose /*T_0_i*/ =
                T_0_enu + T_enu_i;

            state_.submapsGraph.insertEdge(ref_id, this_id, relPose);

            // 2) Add edge to low-level keyframe graph:

            const gtsam::Pose3 deltaPose =
                mrpt::gtsam_wrappers::toPose3(relPose.getPoseMean());

            auto edgeNoise = gtsam::noiseModel::Gaussian::Covariance(
                mrpt::gtsam_wrappers::to_gtsam_se3_cov6_reordering(
                    relPose.cov));

            const auto refKfId = *refSubmap.kf_ids.begin();
            const auto curKfId = *submap.kf_ids.begin();

            {
                const auto p0 =
                    state_.kfGraphValues.at<gtsam::Pose3>(X(refKfId));
                const auto pi =
                    state_.kfGraphValues.at<gtsam::Pose3>(X(curKfId));
                const auto p01pre = mrpt::gtsam_wrappers::toTPose3D(pi) -
                                    mrpt::gtsam_wrappers::toTPose3D(p0);

                MRPT_LOG_DEBUG_STREAM(
                    "FG GNNS edge:\n"
                    "p01: "
                    << p01pre
                    << "\n"
                       "new: "
                    << relPose.getPoseMean().asTPose());
            }

            state_.kfGraphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                X(refKfId), X(curKfId), deltaPose, edgeNoise);

            state_.kfGraphFGRobust
                .emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                    X(refKfId), X(curKfId), deltaPose, edgeNoise);
        }

        if (params_.save_submaps_viz_files)
        {  // Save viz of initial state:
            VizOptions opts;
            auto       glMap = build_submaps_visualization(opts);

            mrpt::opengl::Scene scene;
            scene.insert(glMap);

            scene.saveToFile(
                params_.debug_files_prefix + "_submaps_initial_pre.3Dscene"s);
        }

        // Run an initial LM pass to fit the GNNS measurements:
        optimize_graph();
    }

    if (params_.save_submaps_viz_files)
    {  // Save viz of initial state:
        VizOptions opts;
        auto       glMap = build_submaps_visualization(opts);

        mrpt::opengl::Scene scene;
        scene.insert(glMap);

        scene.saveToFile(
            params_.debug_files_prefix + "_submaps_initial.3Dscene"s);
    }

    // Look for potential loop closures:
    // -----------------------------------------------
    // find next smallest potential loop closure?

    size_t accepted_lcs = 0;

    std::set<std::pair<submap_id_t, submap_id_t>> alreadyChecked;
    // TODO: Consider a finer grade alreadyChecked reset?

    // repeat until checkedCount==0:
    for (size_t lc_loop = 0; lc_loop < 1; lc_loop++)
    {
        size_t checkedCount   = 0;
        bool   anyGraphChange = false;

        PotentialLoopOutput LCs = find_next_loop_closures();

        if (params_.max_number_lc_candidates > 0 &&
            LCs.size() > params_.max_number_lc_candidates)
        {
            // dont shuffle: they are already sorted by expected score
#if 0
            std::random_device rd;
            std::mt19937       g(rd());
            std::shuffle(LCs.begin(), LCs.end(), g);
#endif
            LCs.resize(params_.max_number_lc_candidates);
        }

        // Build a list of affected submaps, including how many times they
        // appear:
        std::map<submap_id_t, size_t> LC_submap_IDs_count;
        for (const auto& lc : LCs)
        {
            LC_submap_IDs_count[lc.smallest_id]++;
            LC_submap_IDs_count[lc.largest_id]++;
        }

        // check all LCs, and accept those that seem valid:
        for (size_t lcIdx = 0; lcIdx < LCs.size(); lcIdx++)
        {
            const auto& lc = LCs.at(lcIdx);

            auto IDs = std::make_pair(lc.smallest_id, lc.largest_id);

            if (alreadyChecked.count(IDs) != 0) continue;

            // a new pair. add it:
            alreadyChecked.insert(IDs);
            checkedCount++;

            MRPT_LOG_INFO_STREAM(
                "LC " << lcIdx << "/" << LCs.size() << ": " << lc.smallest_id
                      << "<=>" << lc.largest_id << " score: " << lc.score
                      << " relPose="
                      << lc.relative_pose_largest_wrt_smallest.mean.asString()
                      << " stds: "
                      << lc.relative_pose_largest_wrt_smallest.cov.asEigen()
                             .diagonal()
                             .array()
                             .sqrt()
                             .eval()
                             .transpose());

            const bool accepted = process_loop_candidate(lc);
            if (accepted)
            {
                anyGraphChange = true;
                accepted_lcs++;
            }

            // free the RAM space of submaps not used any more:
            auto lambdaDereferenceLocalMapOnce = [&](submap_id_t id)
            {
                if (0 == --LC_submap_IDs_count[id])
                {
                    MRPT_LOG_INFO_STREAM(
                        "Freeing memory for submap local map #" << id);
                    state_.submaps[id].local_map.reset();
                };
            };
            lambdaDereferenceLocalMapOnce(lc.smallest_id);
            lambdaDereferenceLocalMapOnce(lc.largest_id);
        }

        if (!checkedCount) break;  // no new LC was checked, we are done.

        // any change to the graph? re-optimize it:
        if (anyGraphChange)
        {
            const auto largestDelta = optimize_graph();

            // re-visit all areas again
            if (largestDelta > 3.0) alreadyChecked.clear();
        }
    }

    if (params_.save_submaps_viz_files)
    {  // Save viz of final state:
        VizOptions opts;
        auto       glMap = build_submaps_visualization(opts);

        mrpt::opengl::Scene scene;
        scene.insert(glMap);

        scene.saveToFile(params_.debug_files_prefix + "_submaps_final.3Dscene");
    }

    // At this point, we have optimized the KFs in state_.keyframesGraph.
    // Now, update all low-level keyframes in the simplemap:
    mrpt::maps::CSimpleMap outSM;

    for (size_t id = 0; id < sm.size(); id++)
    {
        auto& [oldPose, sf, twist] = state_.sm->get(id);

        const auto& newKfGlobalPose = state_.kfGraph_get_pose(id);

        const auto newPose = mrpt::poses::CPose3DPDFGaussian::Create();
        newPose->mean      = newKfGlobalPose;
        newPose->cov.setIdentity();  // TODO! Get cov from optimizer?

        outSM.insert(newPose, sf, twist);
    }

    MRPT_LOG_INFO_STREAM(
        "Overall number of accepted loop-closures: " << accepted_lcs);

    // Overwrite with new SM:
    sm = std::move(outSM);
}

namespace
{
void make_pose_planar(mrpt::poses::CPose3D& p)
{
    p.z(0);
    p.setYawPitchRoll(p.yaw(), .0, .0);
}
void make_pose_planar_pdf(mrpt::poses::CPose3DPDFGaussian& pdf)
{
    make_pose_planar(pdf.mean);
    // z[2]=0, pitch[4]=0, roll[5]
    for (int i = 0; i < 6; i++)
    {
        pdf.cov(i, 2) = 0;
        pdf.cov(2, i) = 0;
        pdf.cov(i, 4) = 0;
        pdf.cov(4, i) = 0;
        pdf.cov(i, 5) = 0;
        pdf.cov(5, i) = 0;
    }
}
}  // namespace

void SimplemapLoopClosure::build_submap_from_kfs_into(
    const std::set<keyframe_id_t>& ids, SubMap& submap)
{
    const keyframe_id_t refFrameId = *ids.begin();

    submap.kf_ids      = ids;
    submap.global_pose = keyframe_pose_in_simplemap(refFrameId);

    const auto invSubmapPose = -submap.global_pose;

    if (params_.assume_planar_world) make_pose_planar(submap.global_pose);

    MRPT_LOG_DEBUG_STREAM(
        "Defining submap #" << submap.id << " with " << ids.size()
                            << " keyframes.");

    // Load the bbox of the frame from the SimpleMap metadata entry:
    // Insert all observations in this submap:

    std::optional<mrpt::math::TBoundingBox> bbox;
    size_t                                  gnnsCount = 0;
    mrpt::maps::CSimpleMap                  subSM;  // aux SM for this submap

    for (const auto& id : submap.kf_ids)
    {
        const auto& [pose, sf, twist] = state_.sm->get(id);

        // Create submap SM, for latter use in GNNS geo-reference:
        auto relPdf = mrpt::poses::CPose3DPDFGaussian::Create();
        relPdf->copyFrom(*pose);
        relPdf->changeCoordinatesReference(invSubmapPose);

        subSM.insert(relPdf, sf, twist);

        // process metadata as embedded YAML "observation":
        if (auto oc =
                sf->getObservationByClass<mrpt::obs::CObservationComment>();
            oc)
        {
            auto yml = mrpt::containers::yaml::FromText(oc->text);

            auto pMin = mrpt::math::TPoint3D::FromString(
                yml["frame_bbox_min"].as<std::string>());
            auto pMax = mrpt::math::TPoint3D::FromString(
                yml["frame_bbox_max"].as<std::string>());

            // transform bbox and extend bbox in local submap coordinates:
            const auto p = keyframe_relative_pose_in_simplemap(id, refFrameId);

            const auto pMinLoc = p.composePoint(pMin);
            const auto pMaxLoc = p.composePoint(pMax);
            if (!bbox)
                bbox = mrpt::math::TBoundingBox::FromUnsortedPoints(
                    pMinLoc, pMaxLoc);
            else
            {
                bbox->updateWithPoint(pMinLoc);
                bbox->updateWithPoint(pMaxLoc);
            }
        }

        // Process GNNS?
        if (auto oG = sf->getObservationByClass<mrpt::obs::CObservationGPS>();
            oG)
        {
            gnnsCount++;
        }

    }  // for each keyframe

    // Try to generate geo-referencing data:
    if (params_.use_gnns && gnnsCount > 2)
    {
        SMGeoReferencingParams geoParams;
        geoParams.fgParams.addHorizontalityConstraints = false;

        geoParams.logger            = this;
        geoParams.geodeticReference = state_.globalGeoRef;

        auto geoResult = simplemap_georeference(subSM, geoParams);

        // decent solution?
        const auto se3Stds = geoResult.geo_ref.T_enu_to_map.cov.asEigen()
                                 .diagonal()
                                 .array()
                                 .sqrt()
                                 .eval();
        const auto angleStds = se3Stds.tail<3>();

        if (geoResult.final_rmse < 1.0)
        {
            // save in submap:

            if (angleStds.maxCoeff() < 0.1)
            {
                // Null pitch & roll since they don't seem reliable:
                auto& p = geoResult.geo_ref.T_enu_to_map.mean;
                p.setYawPitchRoll(p.yaw(), .0, .0);
            }

            submap.geo_ref = geoResult.geo_ref;

            // Use one single global reference frame for all submaps:
            if (!state_.globalGeoRef && submap.geo_ref)
            {
                state_.globalGeoRef         = submap.geo_ref->geo_coord;
                state_.globalGeoRefSubmapId = submap.id;
            }

            // Update the global pose too:
            // T_0_i = (T_enu_0)⁻¹ · T_enu_i (notes picture! pass to paper)
            const auto T_enu_0 = state_.submaps.at(state_.globalGeoRefSubmapId)
                                     .geo_ref->T_enu_to_map;
            auto T_0_enu = -T_enu_0;
            T_0_enu.cov.setZero();  // Ignore uncertainty of this first T

            const auto T_enu_i = submap.geo_ref->T_enu_to_map;
            const auto T_0_i   = T_0_enu + T_enu_i;

            MRPT_LOG_INFO_STREAM(
                "[build_submap_from_kfs_into] ACCEPTING submap #"
                << submap.id
                << " GNNS T_enu_to_map=" << geoResult.geo_ref.T_enu_to_map.mean
                << "\n globalPose=" << T_0_i.mean  //
                << "\n was       =" << submap.global_pose
                << "\n se3Stds   =" << se3Stds.transpose());

            submap.global_pose = T_0_i.getPoseMean();
        }
        else
        {
            MRPT_LOG_INFO_STREAM(
                "[build_submap_from_kfs_into] DISCARDING GNNS solution for "
                "submap #"
                << submap.id
                << " GNNS T_enu_to_map=" << geoResult.geo_ref.T_enu_to_map.mean
                << " se3Stds=" << se3Stds.transpose());
        }
    }

    if (bbox.has_value())
    {
        // Use bbox from SimpleMap metadata annotations:
        submap.bbox = *bbox;

        MRPT_LOG_DEBUG_STREAM(
            "[build_submap_from_kfs_into] Built bbox from metadata: "
            << bbox->asString());
    }
    else
    {
        // We have as input a simplemap without metadata.
        // Just build the whole metric map now:
        auto mm = get_submap_local_map(submap);
        mm.get();  // get future
    }
}

mrpt::poses::CPose3D SimplemapLoopClosure::keyframe_pose_in_simplemap(
    keyframe_id_t kfId) const
{
    const auto& [pose, sf, twist] = state_.sm->get(kfId);
    ASSERT_(pose);
    return pose->getMeanVal();
}

mrpt::poses::CPose3D SimplemapLoopClosure::keyframe_relative_pose_in_simplemap(
    keyframe_id_t kfId, keyframe_id_t referenceKfId) const
{
    return keyframe_pose_in_simplemap(kfId) -
           keyframe_pose_in_simplemap(referenceKfId);
}

void SimplemapLoopClosure::updatePipelineDynamicVariablesForKeyframe(
    const keyframe_id_t id, const keyframe_id_t referenceId,
    const size_t threadIdx)
{
    auto& pts = state_.perThreadState_.at(threadIdx);

    auto& ps = pts.parameter_source;

    const auto& [globalPose, sf, twist] = state_.sm->get(id);

    // Set dynamic variables for twist usage within ICP pipelines
    // (e.g. de-skew methods)
    {
        mrpt::math::TTwist3D twistForIcpVars = {0, 0, 0, 0, 0, 0};
        if (twist) twistForIcpVars = *twist;

        ps.updateVariable("VX", twistForIcpVars.vx);
        ps.updateVariable("VY", twistForIcpVars.vy);
        ps.updateVariable("VZ", twistForIcpVars.vz);
        ps.updateVariable("WX", twistForIcpVars.wx);
        ps.updateVariable("WY", twistForIcpVars.wy);
        ps.updateVariable("WZ", twistForIcpVars.wz);
    }

    // robot pose:
    const auto p = keyframe_relative_pose_in_simplemap(id, referenceId);

    ps.updateVariable("ROBOT_X", p.x());
    ps.updateVariable("ROBOT_Y", p.y());
    ps.updateVariable("ROBOT_Z", p.z());
    ps.updateVariable("ROBOT_YAW", p.yaw());
    ps.updateVariable("ROBOT_PITCH", p.pitch());
    ps.updateVariable("ROBOT_ROLL", p.roll());

    ps.updateVariable("SIGMA_INIT", params_.threshold_sigma_initial);
    ps.updateVariable("SIGMA_FINAL", params_.threshold_sigma_final);

    ps.updateVariable("ESTIMATED_SENSOR_MAX_RANGE", params_.max_sensor_range);

    // This will be overwritten by the actual ICP loop later on,
    // but we need to define all variables before building a local map:
    ps.updateVariable("ICP_ITERATION", 0);

    // Make all changes effective and evaluate the variables now:
    ps.realize();
}

mrpt::opengl::CSetOfObjects::Ptr
    SimplemapLoopClosure::build_submaps_visualization(const VizOptions& p) const
{
    auto glViz = mrpt::opengl::CSetOfObjects::Create();

    // Show graph:
    mrpt::containers::yaml extra_params;
    extra_params["show_ID_labels"] = true;
    extra_params["show_edges"]     = p.show_edges;

    auto glGraph = mrpt::opengl::graph_tools::graph_visualize(
        state_.submapsGraph, extra_params);

    // Show at an elevated height:
    glGraph->setLocation(0, 0, 10);

    glViz->insert(glGraph);

    // Boxes and mini-maps for each submap:
    for (const auto& [id, submap] : state_.submaps)
    {
        auto glSubmap = mrpt::opengl::CSetOfObjects::Create();

        if (p.show_bbox)
        {
            auto glBox = mrpt::opengl::CBox::Create();
            glBox->setWireframe(true);
            auto bbox = submap.bbox;
            glBox->setBoxCorners(bbox.min, bbox.max);
            glSubmap->insert(glBox);
        }
        if (!p.viz_point_layer.empty() && submap.local_map &&
            submap.local_map->layers.count(p.viz_point_layer) != 0)
        {
            auto& m = submap.local_map->layers.at(p.viz_point_layer);
            mp2p_icp::metric_map_t mm;
            mm.layers["dummy"] = m;

            mp2p_icp::render_params_t rp;
            rp.points.allLayers.pointSize = 3.0f;

            glSubmap->insert(mm.get_visualization(rp));
        }

        glSubmap->setPose(submap.global_pose);
        glViz->insert(glSubmap);
    }

    return glViz;
}

SimplemapLoopClosure::PotentialLoopOutput
    SimplemapLoopClosure::find_next_loop_closures() const
{
    using namespace std::string_literals;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "find_next_loop_closure");

    struct InfoPerSubmap
    {
        mrpt::poses::CPose3DPDFGaussian pose;
        size_t                          depth = 0;
    };

    std::multimap<double /*intersectRatio*/, PotentialLoop> potentialLCs;

    // go on thru all nodes as root of Dijkstra:
    for (const auto& [root_id, rootPose] : state_.submapsGraph.nodes)
    {
        mrpt::system::CTimeLoggerEntry tle1(
            profiler_, "find_next_loop_closure.single");

        mrpt::graphs::CDijkstra<typeof(state_.submapsGraph)> dijkstra(
            state_.submapsGraph, root_id);

        using tree_t = mrpt::graphs::CDirectedTree<
            const mrpt::graphs::CNetworkOfPoses3DCov::edge_t*>;

        const tree_t tree = dijkstra.getTreeGraph();

        std::map<submap_id_t, InfoPerSubmap> submapPoses;

        submapPoses[root_id] = {};  // perfect identity pose with zero cov.

        auto lambdaVisitTree = [&](mrpt::graphs::TNodeID const parent,
                                   const tree_t::TEdgeInfo&    edgeToChild,
                                   size_t                      depthLevel)
        {
            auto& ips = submapPoses[edgeToChild.id];

            mrpt::poses::CPose3DPDFGaussian edge = *edgeToChild.data;

            if (params_.assume_planar_world) make_pose_planar_pdf(edge);

            ips.pose  = submapPoses[parent].pose + edge;
            ips.depth = depthLevel;

            MRPT_LOG_DEBUG_STREAM(
                "TREE visit: " << parent << " depth: " << depthLevel
                               << " pose mean=" << ips.pose.mean);
        };

        // run lambda:
        tree.visitDepthFirst(root_id, lambdaVisitTree);

        // Debug:
        if (SAVE_TREES)
        {
            const std::string d = "trees_"s + params_.debug_files_prefix;
            mrpt::system::createDirectory(d);
            const auto sFil = mrpt::format(
                "%s/tree_root_%04u.3Dscene", d.c_str(), (unsigned int)root_id);
            std::cout << "[SAVE_TREES] Saving tree : " << sFil << std::endl;

            mrpt::opengl::Scene scene;

            for (const auto& [id, m] : submapPoses)
            {
                {
                    auto glCorner =
                        mrpt::opengl::stock_objects::CornerXYZSimple(2.5f);
                    glCorner->enableShowName();

                    std::string label = "#"s + std::to_string(id);
                    label += " d="s + std::to_string(m.depth);

                    if (state_.submaps.at(id).geo_ref.has_value())
                        label += " (WITH GPS)"s;

                    glCorner->setName(label);
                    glCorner->setPose(m.pose.mean);
                    scene.insert(glCorner);
                }

                {
                    auto glEllip = mrpt::opengl::CEllipsoid2D::Create();
                    glEllip->setCovMatrix(
                        m.pose.cov.asEigen().block<2, 2>(0, 0));
                    glEllip->setLocation(m.pose.mean.translation());
                    glEllip->setQuantiles(2.0);
                    scene.insert(glEllip);
                }
            }

            scene.saveToFile(sFil);

        }  // end SAVE_TREES

        // Look for all potential overlapping areas between root_id and all
        // other areas:

        const auto& rootBbox = state_.submaps.at(root_id).bbox;

        for (const auto& [submapId, ips] : submapPoses)
        {
            // dont match against myself!
            if (submapId == root_id) continue;

            // we need at least topological distance>=2 for this to be L.C.
            // (except if we are using GNNS edges and one ID is the GNNS
            // reference submap!)
            if (ips.depth <= 1 && (!state_.globalGeoRef.has_value() ||
                                   (submapId != state_.globalGeoRefSubmapId &&
                                    root_id != state_.globalGeoRefSubmapId)))
                continue;  // skip it

            const auto min_id = std::min<submap_id_t>(root_id, submapId);
            const auto max_id = std::max<submap_id_t>(root_id, submapId);

            // TODO(jlbc): finer approach without enlarging bboxes to their
            // global XYZ axis alined boxes:
            // Idea: run a small MonteCarlo run to estimate the likelihood
            // of an overlap for large uncertainties:
            const size_t MC_RUNS =
                mrpt::saturate_val<size_t>(10 * ips.depth, 10, 400);

            mrpt::poses::CPoseRandomSampler sampler;
            sampler.setPosePDF(ips.pose);

            if (PRINT_ALL_SCORES)
            {
                MRPT_LOG_INFO_STREAM(
                    "Relative pose: " << min_id << " <==> " << max_id
                                      << " pose: " << ips.pose);
            }

            double               bestScore = .0;
            mrpt::poses::CPose3D bestRelPose;

            const auto& thisBBox = state_.submaps.at(submapId).bbox;

            for (size_t i = 0; i < MC_RUNS; i++)
            {
                mrpt::poses::CPose3D relPoseSample;
                sampler.drawSample(relPoseSample);

                const auto relativeBBox =
                    thisBBox.compose(relPoseSample.asTPose());

                const auto bboxIntersect = rootBbox.intersection(relativeBBox);

                if (!bboxIntersect.has_value()) continue;  // no overlap at all

                const double intersectRatio =
                    bboxIntersect->volume() /
                    (0.5 * rootBbox.volume() + 0.5 * relativeBBox.volume());

                if (intersectRatio > bestScore)
                {
                    bestScore   = intersectRatio;
                    bestRelPose = relPoseSample;
                }
            }

            if (PRINT_ALL_SCORES)
            {
                MRPT_LOG_INFO_STREAM(
                    "Score for LC: " << min_id << " <==> " << max_id
                                     << " bestScore=" << bestScore
                                     << " topo_depth=" << ips.depth
                                     << " MC_RUNS=" << MC_RUNS);
            }

            if (bestScore <
                params_.min_volume_intersection_ratio_for_lc_candidate)
                continue;

            PotentialLoop lc;
            lc.largest_id           = max_id;
            lc.smallest_id          = min_id;
            lc.topological_distance = ips.depth;
            lc.score                = bestScore;
            if (root_id == min_id)
                lc.relative_pose_largest_wrt_smallest = ips.pose;
            else
            {  // inverse SE(3) relative pose
                ips.pose.inverse(lc.relative_pose_largest_wrt_smallest);
            }

            // for very long loops with too large uncertainty, replace the
            // (probably too bad) relative pose mean with the best one from
            // the MonteCarlo sample above:
            {
                const double std_xy =
                    std::sqrt(ips.pose.cov.block(0, 0, 2, 2).determinant());

                const double submap_size = (thisBBox.max - thisBBox.min).norm();

                const double ratio =
                    submap_size > 0 ? (std_xy / submap_size) : 1.0;

                if (PRINT_ALL_SCORES)
                    MRPT_LOG_INFO_STREAM(
                        "|C(1:2,1:2)|=" << std_xy << " |submap_size|="
                                        << submap_size << " ratio=" << ratio);

#if 0
                if (ratio > 2)
                {
                    // save the original LC too:
                    potentialLCs.emplace(bestScore, lc);

                    // and a new one, keeping the best MC sample, plus some
                    // additional ones:
                    lc.draw_several_samples = true;

                    // and change the mean:
                    if (root_id == min_id)
                        lc.relative_pose_largest_wrt_smallest.mean =
                            bestRelPose;
                    else
                        lc.relative_pose_largest_wrt_smallest.mean =
                            -bestRelPose;
                }
#endif
            }

            potentialLCs.emplace(bestScore, lc);
        }
    }  // end for each root_id

    // debug, print potential LCs:
    for (const auto& [score, lc] : potentialLCs)
    {
        MRPT_LOG_DEBUG_STREAM(
            "Initial potential LC: "
            << lc.smallest_id << " <==> " << lc.largest_id << " score=" << score
            << " topo_depth=" << lc.topological_distance);
    }

    // filter them, and keep the most promising ones, sorted by "score"
    PotentialLoopOutput result;
    for (auto it = potentialLCs.rbegin(); it != potentialLCs.rend(); ++it)
    {
        const auto& [score, lc] = *it;

        result.push_back(lc);
#if 0
        // only store map entries for the "smallest_id" of each pair to
        // avoid duplicated checks:
        if (result.count(lc.largest_id) != 0) continue;

        auto& rs = result[lc.smallest_id];
        rs.emplace_back(lc);
#endif
    }

    // debug, print potential LCs:
    for (const auto& lc : result)
    {
        MRPT_LOG_INFO_STREAM(
            "[find_lc] Potential LC: "  //
            << lc.smallest_id << " <==> " << lc.largest_id
            << " topo_depth=" << lc.topological_distance << " relPose: "
            << lc.relative_pose_largest_wrt_smallest.mean.asString()
            << " score: " << lc.score);
    }

    return result;
}

bool SimplemapLoopClosure::process_loop_candidate(const PotentialLoop& lc)
{
    using namespace std::string_literals;
    using namespace mrpt::literals;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "process_loop_candidate");

    // Apply ICP between the two submaps:
    const auto  idGlobal     = lc.smallest_id;
    const auto& submapGlobal = state_.submaps.at(idGlobal);

    const auto  idLocal     = lc.largest_id;
    const auto& submapLocal = state_.submaps.at(idLocal);

    auto mapGlobalFut = get_submap_local_map(submapGlobal);
    auto mapLocalFut  = get_submap_local_map(submapLocal);

    const auto& mapGlobal = mapGlobalFut.get();
    const auto& mapLocal  = mapLocalFut.get();

    ASSERT_(mapGlobal);
    ASSERT_(mapLocal);

    const auto& pcs_global = *mapGlobal;
    const auto& pcs_local  = *mapLocal;

    MRPT_LOG_DEBUG_STREAM(
        "LC candidate: relPose=" << lc.relative_pose_largest_wrt_smallest);

    if (SAVE_LCS)
    {
        static int        nLoop = 0;
        const std::string d     = "lcs_"s + params_.debug_files_prefix;
        mrpt::system::createDirectory(d);
        const auto sDir = mrpt::format(
            "%s/loop_%04i_g%03u_l%03u", d.c_str(), nLoop++,
            (unsigned int)*pcs_global.id, (unsigned int)*pcs_local.id);
        mrpt::system::createDirectory(sDir);
        std::cout << "[LC] Saving loop closure files to: " << sDir << "\n";

        pcs_global.save_to_file(mrpt::system::pathJoin({sDir, "global.mm"}));
        pcs_local.save_to_file(mrpt::system::pathJoin({sDir, "local.mm"}));

        std::ofstream f(
            mrpt::system::pathJoin({sDir, "init_pose_local_wrt_global.txt"}));
        f << lc.relative_pose_largest_wrt_smallest;
    }

    const mrpt::math::TPose3D initGuess =
        lc.relative_pose_largest_wrt_smallest.mean.asTPose();

    bool atLeastOneGoodIcp = false;

    auto lambdaAddIcpEdge =
        [&](const mrpt::poses::CPose3D& icpRelPose, const double icpQuality)
    {
        if (!state_.submapsGraph.edgeExists(idGlobal, idLocal))
        {
            mrpt::poses::CPose3DPDFGaussian newEdge;
            newEdge.mean = icpRelPose;
            newEdge.cov.setIdentity();  // Not used anyway

            state_.submapsGraph.insertEdge(idGlobal, idLocal, newEdge);
        }

        // and to the low-level graph too:
        const gtsam::Pose3 deltaPose =
            mrpt::gtsam_wrappers::toPose3(icpRelPose);

        using gtsam::symbol_shorthand::X;

        double edge_std_xyz = icpRelPose.norm() * 0.5 * 1e-2;  // 0.5% RTE
        double edge_std_ang = mrpt::DEG2RAD(0.5);

        // Use a variable variance depending on the ICP quality:
        ASSERT_(params_.icp_edge_worst_multiplier > 1.0);

        double std_multiplier = params_.icp_edge_worst_multiplier -
                                (params_.icp_edge_worst_multiplier - 1.0) *
                                    (icpQuality - params_.min_icp_goodness) /
                                    params_.min_icp_goodness;

        edge_std_xyz *= std_multiplier;
        edge_std_ang *= std_multiplier;

        const double icp_edge_robust_param = params_.icp_edge_robust_param;

        gtsam::Vector6 sigmas;
        sigmas << edge_std_ang, edge_std_ang, edge_std_ang,  //
            edge_std_xyz, edge_std_xyz, edge_std_xyz;

        auto icpNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

        gtsam::noiseModel::Base::shared_ptr icpRobNoise =
            gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                    icp_edge_robust_param),
                icpNoise);

        // Non-robust graph:
        state_.kfGraphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(*submapGlobal.kf_ids.begin()), X(*submapLocal.kf_ids.begin()),
            deltaPose, icpNoise);

        // Robust graph:
        state_.kfGraphFGRobust
            .emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                X(*submapGlobal.kf_ids.begin()), X(*submapLocal.kf_ids.begin()),
                deltaPose, icpRobNoise);

        atLeastOneGoodIcp = true;
    };

    std::vector<mrpt::math::TPose3D> initGuesses;

    // 1) gross relocalization:
    if (params_.do_first_gross_relocalize)
    {
        // Build a reference map
        mp2p_icp::metric_map_t refMap;
        auto                   refPtsMap =
            pcs_global.layers.at("points_to_register");  // "localmap"
        refMap.layers["raw"] = refPtsMap;

        // These options may be loaded from an INI file, etc.
        auto& likOpts =
            std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(refPtsMap)
                ->likelihoodOptions;

        likOpts.max_corr_distance = 1.5;
        likOpts.decimation        = 1;
        likOpts.sigma_dist        = mrpt::square(0.2);  // variance

        // query observation:
        mrpt::obs::CSensoryFrame querySf;
        auto obs2        = mrpt::obs::CObservationPointCloud::Create();
        obs2->pointcloud = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
            pcs_local.layers.at("points_to_register"));
        querySf.insert(obs2);

        mola::RelocalizationLikelihood_SE2::Input in;
        in.corner_min = mrpt::math::TPose2D(initGuess);
        in.corner_max = mrpt::math::TPose2D(initGuess);

        in.corner_min.x -= 10.0;
        in.corner_max.x += 10.0;

        in.corner_min.y -= 10.0;
        in.corner_max.y += 10.0;

        in.corner_min.phi -= 5.0_deg;
        in.corner_max.phi += 5.0_deg;

        in.observations   = querySf;
        in.reference_map  = refMap;
        in.resolution_xy  = 1.0;
        in.resolution_phi = 5.0_deg;

        MRPT_LOG_DEBUG_STREAM(
            "[Relocalize SE(2)] About to run with localPts="
            << obs2->pointcloud->size() << " globalPts=" << refMap.size()
            << " corner_min=" << in.corner_min
            << " corner_max=" << in.corner_max);

        const auto out = mola::RelocalizationLikelihood_SE2::run(in);

        // search top candidates:
        auto bestPoses = mola::find_best_poses_se2(out.likelihood_grid, 0.95);

        constexpr size_t MAX_BEST_POSES = 10;

        while (bestPoses.size() > MAX_BEST_POSES)
            bestPoses.erase(--bestPoses.end());

        MRPT_LOG_INFO_STREAM(
            "[Relocalize SE(2)] time_cost: " <<  //
            out.time_cost << " Top poses: " << bestPoses.size());

        for (const auto& [score, pose2D] : bestPoses)
        {
            auto pose  = mrpt::math::TPose3D(pose2D);
            pose.z     = initGuess.z;
            pose.roll  = initGuess.roll;
            pose.pitch = initGuess.pitch;
            initGuesses.push_back(pose);
        }

#if 0
    const auto nPhis = out.likelihood_grid.getSizePhi();
    std::vector<mrpt::math::CMatrixDouble> slices;
    slices.resize(nPhis);
    double maxLik = 0, minLik = 0;

    for (size_t iPhi = 0; iPhi < nPhis; iPhi++) {
      const double phi = out.likelihood_grid.idx2phi(iPhi);

      auto &s = slices.at(iPhi);
      out.likelihood_grid.getAsMatrix(phi, s);

      mrpt::keep_max(maxLik, s.maxCoeff());
      mrpt::keep_min(minLik, s.minCoeff());
    }

    // normalize all:
    for (size_t iPhi = 0; iPhi < nPhis; iPhi++) {
      auto &s = slices.at(iPhi);
      s -= minLik;
      s *= 1.0 / (maxLik - minLik);
    }

    // save:
    for (size_t iPhi = 0; iPhi < nPhis; iPhi++) {
      auto &s = slices.at(iPhi);

      s.saveToTextFile(mrpt::format("/tmp/slice_gid_%02u_lid_%02u_phi_%02u.txt",
                                    static_cast<unsigned int>(idGlobal),
                                    static_cast<unsigned int>(idLocal),
                                    static_cast<unsigned int>(iPhi)));

      mrpt::img::CImage im;
      im.setFromMatrix(s, true /*normalized [0,1]*/);
      bool savedOk = im.saveToFile(mrpt::format(
          "/tmp/slice_gid_%02u_lid_%02u_phi_%02u.png",
          static_cast<unsigned int>(idGlobal),
          static_cast<unsigned int>(idLocal), static_cast<unsigned int>(iPhi)));

      ASSERT_(savedOk);
    }

#endif
    }
    else if (params_.do_montecarlo_icp)
    {
        // Convert pose with uncertainty SE(3) -> SE(2)
        mrpt::poses::CPosePDFGaussian pdf_SE2;
        pdf_SE2.copyFrom(lc.relative_pose_largest_wrt_smallest);

        mola::RelocalizationICP_SE2::Input in;
        in.icp_minimum_quality = params_.min_icp_goodness;

        // Use multi-thread ICP:
        for (size_t threadIdx = 0; threadIdx < state_.perThreadState_.size();
             threadIdx++)
        {
            auto& pts = state_.perThreadState_.at(threadIdx);

            // (this defines the local robot pose on the submap)
            updatePipelineDynamicVariablesForKeyframe(
                *submapLocal.kf_ids.begin(), *submapLocal.kf_ids.begin(),
                threadIdx);

            in.icp_pipeline.push_back(pts.icp);
        }

        // All threads with the same params:
        in.icp_parameters = params_.icp_parameters;

        double       std_x   = std::sqrt(pdf_SE2.cov(0, 0));
        double       std_y   = std::sqrt(pdf_SE2.cov(1, 1));
        const double std_yaw = std::sqrt(pdf_SE2.cov(2, 2));

        const double maxMapLenght =
            (submapLocal.bbox.max - submapLocal.bbox.min).norm();

        mrpt::saturate(std_x, 1.0, 0.25 * maxMapLenght);
        mrpt::saturate(std_y, 1.0, 0.25 * maxMapLenght);

        in.initial_guess_lattice.corner_min = {
            pdf_SE2.mean.x() - 3 * std_x,
            pdf_SE2.mean.y() - 3 * std_y,
            pdf_SE2.mean.phi() - 3 * std_yaw,
        };
        in.initial_guess_lattice.corner_max = {
            pdf_SE2.mean.x() + 3 * std_x,
            pdf_SE2.mean.y() + 3 * std_y,
            pdf_SE2.mean.phi() + 3 * std_yaw,
        };
        in.initial_guess_lattice.resolution_xy = mrpt::saturate_val<double>(
            std::max(std_x, std_y) * 3, 2.0, 0.2 * maxMapLenght);
        in.initial_guess_lattice.resolution_phi =
            mrpt::saturate_val<double>(std_yaw * 3, 10.0_deg, 30.0_deg);

        in.output_lattice.resolution_xyz   = 0.35;
        in.output_lattice.resolution_yaw   = 5.0_deg;
        in.output_lattice.resolution_pitch = 5.0_deg;
        in.output_lattice.resolution_roll  = 5.0_deg;

        in.reference_map = pcs_global;
        in.local_map     = pcs_local;

        in.on_progress_callback =
            [&](const mola::RelocalizationICP_SE2::ProgressFeedback& fb)
        {
            MRPT_LOG_INFO_STREAM(
                "[Relocalize SE(2)] Progress " << fb.current_cell << "/"
                                               << fb.total_cells);
        };

        MRPT_LOG_INFO_STREAM(
            "[Relocalize SE(2)] About to run with "
            << " corner_min=" << in.initial_guess_lattice.corner_min
            << " corner_max=" << in.initial_guess_lattice.corner_max);

        const auto out = mola::RelocalizationICP_SE2::run(in);

        // search top candidates:

        // Take the voxel with the largest number of poses:
        std::map<size_t, mrpt::math::TPose3D> bestVoxels;

        out.found_poses.visitAllVoxels(
            [&](const mola::HashedSetSE3::global_index3d_t&,
                const mola::HashedSetSE3::VoxelData& v)
            {
                if (v.poses().empty()) return;
                bestVoxels[v.poses().size()] = v.poses().front();
            });

        if (!bestVoxels.empty())
        {
            const auto& bestPose = bestVoxels.rbegin()->second;
            lambdaAddIcpEdge(mrpt::poses::CPose3D(bestPose), 1.0);

            MRPT_LOG_INFO_STREAM(
                "[Relocalize SE(2)] Result has "
                << out.found_poses.voxels().size()
                << " voxels, most populated one |V|="
                << bestVoxels.rbegin()->first << " bestPose: " << bestPose);

            return true;  // done
        }
    }

    // Default:
    if (initGuesses.empty())
    {
        // do not re-localize, just start with the first initial guess:
        initGuesses.push_back(initGuess);

        if (0)  // lc.draw_several_samples)
        {
            const auto sigmas =
                lc.relative_pose_largest_wrt_smallest.cov.asEigen()
                    .diagonal()
                    .array()
                    .sqrt()
                    .eval();

            const double std_x = sigmas[0];
            const double std_y = sigmas[1];
            // const double std_yaw = sigmas[3];

            for (int ix = -2; ix < 3; ix += 2)
            {
                for (int iy = -2; iy < 3; iy += 2)
                {
                    if (ix == 0 && iy == 0)
                        continue;  // center already added above

                    auto p = initGuess;
                    p.x += ix * std_x;
                    p.y += iy * std_y;
                    // p.yaw += rng.drawGaussian1D(.0, stdYaw);

                    initGuesses.push_back(p);
                }
            }

#if 0 
           for (size_t i = 0; i < 20; i++)
            {
                auto p = initGuess;
                p.x += rng.drawGaussian1D(.0, stdXY);
                p.y += rng.drawGaussian1D(.0, stdXY);
                p.yaw += rng.drawGaussian1D(.0, stdYaw);

                initGuesses.push_back(p);
            }
#endif
        }
    }

    // 2) refine with ICP:
    // Goal: keep the best one.

    std::map<double /*quality*/, mp2p_icp::Results> icp_results;

    for (const auto& initPose : initGuesses)
    {
        mp2p_icp::Results icp_result;

        // Assume we are running this part in single thread (!)
        const size_t threadIdx = 0;

        auto& pts = state_.perThreadState_.at(threadIdx);

        // (this defines the local robot pose on the submap)
        updatePipelineDynamicVariablesForKeyframe(
            *submapLocal.kf_ids.begin(), *submapLocal.kf_ids.begin(),
            threadIdx);

        pts.icp->align(
            pcs_local, pcs_global, initPose, params_.icp_parameters,
            icp_result);

        MRPT_LOG_INFO_FMT(
            "ICP: goodness=%.02f%% iters=%u pose=%s "
            "termReason=%s for initPose=%s",
            100.0 * icp_result.quality,
            static_cast<unsigned int>(icp_result.nIterations),
            icp_result.optimal_tf.getMeanVal().asString().c_str(),
            mrpt::typemeta::enum2str(icp_result.terminationReason).c_str(),
            initPose.asString().c_str());

        // keep the best:
        if (icp_result.quality < params_.min_icp_goodness) continue;

        icp_results[icp_result.quality] = icp_result;

        // good enough to skip the rest of init guesses?
        if (icp_result.quality > 0.95)
        {
            MRPT_LOG_INFO(
                "ICP was GOOD enough to skip the rest of initial seeds");
            break;
        }
    }

    // add a new graph edge:
    if (!icp_results.empty())
    {
        const auto& best = icp_results.rbegin()->second;
        lambdaAddIcpEdge(best.optimal_tf.mean, best.quality);
    }

    return atLeastOneGoodIcp;
}

mrpt::poses::CPose3D SimplemapLoopClosure::State::kfGraph_get_pose(
    const keyframe_id_t id) const
{
    using gtsam::symbol_shorthand::X;
    return mrpt::poses::CPose3D(
        mrpt::gtsam_wrappers::toTPose3D(kfGraphValues.at<gtsam::Pose3>(X(id))));
}

std::future<mp2p_icp::metric_map_t::Ptr>
    SimplemapLoopClosure::get_submap_local_map(const SubMap& submap)
{
    const size_t threadIdx = submap.id % state_.perThreadState_.size();

    auto fut = threads_.enqueue(
        [this, threadIdx](const SubMap* m)
        {
            // ensure only 1 thread is running for each per-thread data:
            auto lck =
                mrpt::lockHelper(state_.perThreadState_.at(threadIdx).mtx);

            auto mm = this->impl_get_submap_local_map(*m);

            return mm;
        },
        &submap);
    return fut;
}

mp2p_icp::metric_map_t::Ptr SimplemapLoopClosure::impl_get_submap_local_map(
    const SubMap& submap)
{
    if (submap.local_map) return submap.local_map;

    const size_t threadIdx = submap.id % state_.perThreadState_.size();

    const keyframe_id_t refFrameId = *submap.kf_ids.begin();

    auto& pts = state_.perThreadState_.at(threadIdx);

    // Insert all observations in this submap:
    for (const auto& id : submap.kf_ids)
    {
        // (this defines the local robot pose on the submap)
        updatePipelineDynamicVariablesForKeyframe(id, refFrameId, threadIdx);

        // now that we have valid dynamic variables, check if we need to
        // create the submap on the first KF:
        if (!submap.local_map)
        {
            // Create empty local map:
            submap.local_map = mp2p_icp::metric_map_t::Create();

            // and populate with empty metric maps:
            pts.parameter_source.realize();

            mrpt::obs::CSensoryFrame dummySF;
            {
                auto obs = mrpt::obs::CObservationPointCloud::Create();
                dummySF.insert(obs);
            }

            mp2p_icp_filters::apply_generators(
                pts.local_map_generators, dummySF, *submap.local_map);
        }

        // insert observations from the keyframe:
        // -------------------------------------------

        // Extract points from observation:
        auto observation = mp2p_icp::metric_map_t::Create();

        const auto& [pose, sf, twist] = state_.sm->get(id);

#if 0
        MRPT_LOG_DEBUG_STREAM(
            "Processing KF#" << id << " with |SF|=" << sf->size());
#endif

        // Some frames may be empty:
        if (sf->empty()) continue;
        if (sf->size() == 1 &&
            IS_CLASS(
                *sf->getObservationByIndex(0), mrpt::obs::CObservationComment))
            continue;

        mrpt::system::CTimeLoggerEntry tle0(
            profiler_, "add_submap_from_kfs.apply_generators");

        for (const auto& o : *sf)
            mp2p_icp_filters::apply_generators(
                pts.obs_generators, *o, *observation);

        tle0.stop();

        // Filter/segment the point cloud (optional, but normally will be
        // present):
        mrpt::system::CTimeLoggerEntry tle1(
            profiler_, "add_submap_from_kfs.filter_pointclouds");

        mp2p_icp_filters::apply_filter_pipeline(
            pts.pc_filter, *observation, profiler_);

        tle1.stop();

        // Merge "observation_layers_to_merge_local_map" in local map:
        // ---------------------------------------------------------------
        mrpt::system::CTimeLoggerEntry tle3(
            profiler_, "add_submap_from_kfs.update_local_map");

        // Input  metric_map_t: observation
        // Output metric_map_t: state_.local_map

        // 1/4: temporarily make a (shallow) copy of the observation layers
        // into the local map:
        ASSERT_(submap.local_map);
        for (const auto& [lyName, lyMap] : observation->layers)
        {
            ASSERTMSG_(
                submap.local_map->layers.count(lyName) == 0,
                mrpt::format(
                    "Error: local map layer name '%s' collides with one of "
                    "the observation layers, please use different layer "
                    "names.",
                    lyName.c_str()));

            submap.local_map->layers[lyName] = lyMap;  // shallow copy
        }

        // 2/4: Make sure dynamic variables are up-to-date,
        // in particular, [robot_x, ..., robot_roll]
        // already done above: updatePipelineDynamicVariables();

        // 3/4: Apply pipeline
        mp2p_icp_filters::apply_filter_pipeline(
            pts.obs2map_merge, *submap.local_map, profiler_);

        // 4/4: remove temporary layers:
        for (const auto& [lyName, lyMap] : observation->layers)
            submap.local_map->layers.erase(lyName);

        tle3.stop();
    }  // end for each keyframe ID

    mp2p_icp_filters::apply_filter_pipeline(
        pts.submap_final_filter, *submap.local_map, profiler_);

    // add metadata to local map (for generated debug .icplog files):
    submap.local_map->label = params_.debug_files_prefix;
    submap.local_map->id    = submap.id;

    // Add geo-referencing, if it exists:
    if (submap.geo_ref) submap.local_map->georeferencing = submap.geo_ref;

    // Actual bbox: from point cloud layer:
    std::optional<mrpt::math::TBoundingBoxf> theBBox;

    for (const auto& [name, map] : submap.local_map->layers)
    {
        const auto* ptsMap = mp2p_icp::MapToPointsMap(*map);
        if (!ptsMap || ptsMap->empty()) continue;

        auto bbox = ptsMap->boundingBox();
        if (!theBBox)
            theBBox = bbox;
        else
            theBBox = theBBox->unionWith(bbox);
    }

    std::stringstream debugInfo;
    debugInfo << "submap #" << submap.id << " with " << submap.kf_ids.size()
              << " KFs, local_map: " << submap.local_map->contents_summary();

    if (!theBBox.has_value())
    {
        THROW_EXCEPTION_FMT("no map bbox (!): %s", debugInfo.str().c_str());
    }

    submap.bbox.min = theBBox->min.cast<double>();
    submap.bbox.max = theBBox->max.cast<double>();

    MRPT_LOG_DEBUG_STREAM("Done. Submap metric map: " << debugInfo.str());

    return submap.local_map;
}

double SimplemapLoopClosure::optimize_graph()
{
    // low-level KF graph:
    auto lmParams = gtsam::LevenbergMarquardtParams::LegacyDefaults();

    // Pass 1
    const double errorInit1 = state_.kfGraphFG.error(state_.kfGraphValues);

    gtsam::LevenbergMarquardtOptimizer lm1(
        state_.kfGraphFG, state_.kfGraphValues);
    const auto& optimalValues1 = lm1.optimize();

    const double errorEnd1 = state_.kfGraphFG.error(optimalValues1);

    // Pass 2
    const double errorInit2 = state_.kfGraphFGRobust.error(optimalValues1);

    gtsam::LevenbergMarquardtOptimizer lm2(
        state_.kfGraphFGRobust, optimalValues1);
    const auto& optimalValues2 = lm2.optimize();

    state_.kfGraphValues = optimalValues2;

    const double errorEnd2 = state_.kfGraphFGRobust.error(optimalValues2);

    // Update submaps global pose:
    double largestDelta = 0;

    for (auto& [submapId, submap] : state_.submaps)
    {
        const auto  refId      = *submap.kf_ids.begin();
        const auto& newPose    = state_.kfGraph_get_pose(refId);
        auto&       targetPose = submap.global_pose;
        const auto  deltaPose  = (targetPose - newPose).translation().norm();
        mrpt::keep_max(largestDelta, deltaPose);

        MRPT_LOG_DEBUG_STREAM(
            "Optimized refPose of submap #"
            << submapId << ":\n old=" << targetPose.asTPose()
            << "\n new=" << newPose.asTPose());

        targetPose = newPose;
    }

    MRPT_LOG_INFO_STREAM(
        "***** Graph re-optimized in "
        << lm1.iterations() << "/" << lm2.iterations()
        << " iters, ERROR: 1st PASS:" << errorInit1 << " ==> " << errorEnd1
        << " / 2nd PASS: " << errorInit2 << " ==> " << errorEnd2
        << " largestDelta=" << largestDelta);

    return largestDelta;
}
