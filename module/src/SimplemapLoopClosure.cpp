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

#include <mola_relocalization/relocalization.h>
#include <mola_sm_loop_closure/SimplemapLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>

// MRPT graph-slam:
#include <mrpt/graphs/dijkstra.h>
//#include <mrpt/graphslam/levmarq.h> // replaced by GTSAM

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

SimplemapLoopClosure::SimplemapLoopClosure()
{
    mrpt::system::COutputLogger::setLoggerName("SimplemapLoopClosure");
}

SimplemapLoopClosure::~SimplemapLoopClosure() = default;

void SimplemapLoopClosure::initialize(const mrpt::containers::yaml& c)
{
    MRPT_TRY_START

    // Load params:
    const auto cfg = c["params"];

    if (cfg["lidar_sensor_labels"].isSequence())
    {
        for (const auto& sl : cfg["lidar_sensor_labels"].asSequence())
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
    YAML_LOAD_REQ(params_, submap_max_length, double);
    YAML_LOAD_OPT(params_, do_first_gross_relocalize, bool);

    YAML_LOAD_REQ(params_, threshold_sigma_initial, double);
    YAML_LOAD_REQ(params_, threshold_sigma_final, double);
    YAML_LOAD_REQ(params_, max_sensor_range, double);

    YAML_LOAD_OPT(
        params_, min_volume_intersection_ratio_for_lc_candidate, double);

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
        // Create, and copy my own verbosity level:
        pts.obs2map_merge = mp2p_icp_filters::filter_pipeline_from_yaml(
            c["insert_observation_into_local_map"], this->getMinLoggingLevel());

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            pts.obs2map_merge, pts.parameter_source);

        ASSERT_(!pts.obs2map_merge.empty());

        // Create lidar segmentation algorithm:
        // Observation -> map generator:
        if (c.has("observations_generator") &&
            !c["observations_generator"].isNullNode())
        {
            // Create, and copy my own verbosity level:
            pts.obs_generators = mp2p_icp_filters::generators_from_yaml(
                c["observations_generator"], this->getMinLoggingLevel());
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
            // Create, and copy my own verbosity level:
            pts.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(
                c["observations_filter"], this->getMinLoggingLevel());

            // Attach to the parameter source for dynamic parameters:
            mp2p_icp::AttachToParameterSource(
                pts.pc_filter, pts.parameter_source);
        }

        // Local map generator:
        if (c.has("localmap_generator") &&
            !c["localmap_generator"].isNullNode())
        {
            // Create, and copy my own verbosity level:
            pts.local_map_generators = mp2p_icp_filters::generators_from_yaml(
                c["localmap_generator"], this->getMinLoggingLevel());
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
            c["submap_final_filter"], this->getMinLoggingLevel());

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            pts.submap_final_filter, pts.parameter_source);

    }  // end for threadIdx
    tlePcInit.stop();

    state_.initialized = true;

    MRPT_TRY_END
}

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
        for (size_t i = 0; i < sm.size(); i++)
        {
            pendingKFs.insert(i);

            const auto pose_i_local =
                keyframe_relative_pose_in_simplemap(i, *pendingKFs.begin());

            const auto& [pose_i, sf_i, twist_i] = state_.sm->get(i);

            // don't cut a submap while we are processing empty SFs since we
            // don't know for how long it will take and we might end up with a
            // totally empty final submap
            if (sf_i->empty()) continue;

            if (pose_i_local.translation().norm() >= params_.submap_max_length)
            {
                detectedSubMaps.emplace_back(pendingKFs);
                pendingKFs.clear();
            }
        }
        // remaining ones?
        if (!pendingKFs.empty()) detectedSubMaps.emplace_back(pendingKFs);
    }

    // process pending submap creation, in parallel threads:
    mrpt::WorkerThreadsPool        threads(state_.perThreadState_.size());
    std::vector<std::future<void>> futures;
    const size_t                   nSubMaps = detectedSubMaps.size();

    for (submap_id_t submapId = 0; submapId < nSubMaps; submapId++)
    {
        const size_t threadIdx = submapId % state_.perThreadState_.size();

        // Modify the submaps[] std::map here in this main thread:
        SubMap& submap = state_.submaps[submapId];
        submap.id      = submapId;

        // then process the observations in parallel:
        auto fut = threads.enqueue(
            [this, threadIdx, submapId, nSubMaps](
                std::set<keyframe_id_t> ids, SubMap* m)
            {
                // ensure only 1 thread is running for each per-thread data:
                auto lck =
                    mrpt::lockHelper(state_.perThreadState_.at(threadIdx).mtx);

                build_submap_from_kfs_into(ids, *m, threadIdx);
                MRPT_LOG_DEBUG_STREAM(
                    "Done with submap #" << submapId << " / " << nSubMaps);
            },
            detectedSubMaps.at(submapId), &submap);

        futures.emplace_back(std::move(fut));
    }

    // wait for all them:
    for (auto& f : futures) f.get();

#if 0
    // Debug: save all local maps:
    for (const auto& [id, submap] : state_.submaps)
    {
        ASSERT_(submap.local_map);
        submap.local_map->save_to_file(
            mrpt::format("submap_%03u.mm", static_cast<unsigned int>(id)));
    }
#endif

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

                // Note: this uncertainty for the SUBMAPS graph is actually
                // not used. The important one is the cov in the KEYFRAMES
                // graph.
                relPose.cov.setDiagonal(0.10);

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
            const gtsam::Pose3 p =
                mrpt::gtsam_wrappers::toPose3(keyframe_pose_in_simplemap(id));

            state_.kfGraphValues.insert(X(id), p);

            // anchor for first KF:
            if (state_.kfGraphFG.empty())
            {
                state_.kfGraphFG
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

        state_.kfGraphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);
    }

#if 1
    // Save viz of initial state:
    {
        VizOptions opts;
        auto       glMap = build_submaps_visualization(opts);

        mrpt::opengl::Scene scene;
        scene.insert(glMap);

        scene.saveToFile(
            params_.debug_files_prefix + "_submaps_initial.3Dscene"s);
    }
#endif

    // Look for potential loop closures:
    // -----------------------------------------------
    // find next smallest potential loop closure?

    std::set<std::pair<submap_id_t, submap_id_t>> alreadyChecked;
    // TODO: Consider a finer grade alreadyChecked reset?

    // repeat until checkedCount==0:
    for (size_t lc_loop = 0; lc_loop < 1; lc_loop++)
    {
        size_t checkedCount   = 0;
        bool   anyGraphChange = false;

        const PotentialLoopOutput& LCs = find_next_loop_closures();

        // check them all, and accept those that seem valid:
        for (size_t lcIdx = 0; lcIdx < LCs.size(); lcIdx++)
        {
            const auto& lc = LCs.at(lcIdx);

            auto IDs = std::make_pair(lc.smallest_id, lc.largest_id);

            if (alreadyChecked.count(IDs) != 0) continue;

            // a new pair. add it:
            alreadyChecked.insert(IDs);
            checkedCount++;

            MRPT_LOG_INFO_STREAM(
                "Considering potential LC "
                << lcIdx << "/" << LCs.size() << ": " << lc.smallest_id << "<=>"
                << lc.largest_id << " score: " << lc.score);

            const bool accepted = process_loop_candidate(lc);
            if (accepted) anyGraphChange = true;
        }
        if (!checkedCount) break;  // no new LC was checked, we are done.

        // any change to the graph? re-optimize it:
        if (anyGraphChange)
        {
            // low-level KF graph:
            auto lmParams = gtsam::LevenbergMarquardtParams::LegacyDefaults();

            const double errorInit =
                state_.kfGraphFG.error(state_.kfGraphValues);

            gtsam::LevenbergMarquardtOptimizer lm(
                state_.kfGraphFG, state_.kfGraphValues);
            const auto& optimalValues = lm.optimize();

            state_.kfGraphValues = optimalValues;

            const double errorEnd = state_.kfGraphFG.error(optimalValues);

            // Update submaps global pose:
            double largestDelta = 0;

            for (auto& [submapId, submap] : state_.submaps)
            {
                const auto  refId      = *submap.kf_ids.begin();
                const auto& newPose    = state_.kfGraph_get_pose(refId);
                auto&       targetPose = submap.global_pose;
                const auto  deltaPose =
                    (targetPose - newPose).translation().norm();
                mrpt::keep_max(largestDelta, deltaPose);

                targetPose = newPose;
            }

            MRPT_LOG_INFO_STREAM(
                "***** Graph re-optimized in "
                << lm.iterations() << " iters, ERROR=" << errorInit << " ==> "
                << errorEnd << " largestDelta=" << largestDelta);

            // re-visit all areas again
            if (largestDelta > 3.0) alreadyChecked.clear();
        }
    }

#if 1
    // Save viz of final state:
    {
        VizOptions opts;
        auto       glMap = build_submaps_visualization(opts);

        mrpt::opengl::Scene scene;
        scene.insert(glMap);

        scene.saveToFile(params_.debug_files_prefix + "_submaps_final.3Dscene");
    }
#endif

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

    // Overwrite with new SM:
    sm = std::move(outSM);
}

void SimplemapLoopClosure::build_submap_from_kfs_into(
    const std::set<keyframe_id_t>& ids, SubMap& submap, const size_t threadIdx)
{
    const keyframe_id_t refFrameId = *ids.begin();

    submap.kf_ids      = ids;
    submap.global_pose = keyframe_pose_in_simplemap(refFrameId);

    MRPT_LOG_INFO_STREAM(
        "Populating submap #" << submap.id << " with " << ids.size()
                              << " keyframes... (threadIdx=" << threadIdx
                              << ")");

    auto& pts = state_.perThreadState_.at(threadIdx);

    // Insert all observations in this submap:
    for (const auto& id : ids)
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

        MRPT_LOG_DEBUG_STREAM(
            "Processing KF#" << id << " with |SF|=" << sf->size());

        // Some frames may be empty:
        if (sf->empty()) continue;

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
                    "the "
                    "observation layers, please use different layer names.",
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

    // add metadata:
    submap.local_map->id = submap.id;

    // Bbox: from point cloud layer:
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

    ASSERT_(theBBox.has_value());

    submap.bbox.min = theBBox->min.cast<double>();
    submap.bbox.max = theBBox->max.cast<double>();

    MRPT_LOG_DEBUG_STREAM(
        "Done. Submap metric map: " << submap.local_map->contents_summary());
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
        if (!p.viz_point_layer.empty() &&
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
            ips.pose  = submapPoses[parent].pose + *edgeToChild.data;
            ips.depth = depthLevel;

            MRPT_LOG_DEBUG_STREAM(
                "TREE visit: " << parent << " depth: " << depthLevel
                               << " pose mean=" << ips.pose.mean);
        };

        tree.visitDepthFirst(root_id, lambdaVisitTree);

        // Look for all potential overlapping areas between root_id and all
        // other areas:
        // TODO(jlbc): Use GNNS info!!

        const auto& rootBbox = state_.submaps.at(root_id).bbox;

        for (const auto& [submapId, ips] : submapPoses)
        {
            // dont match against myself!
            if (submapId == root_id) continue;

            // we need at least topological distance>=2 for this to be L.C.
            if (ips.depth <= 1) continue;

            // TODO(jlbc): finer approach without enlarging bboxes to their
            // global XYZ axis alined boxes:
            const auto relativeBBox = state_.submaps.at(submapId).bbox.compose(
                ips.pose.mean.asTPose());

            const auto bboxIntersect = rootBbox.intersection(relativeBBox);

            if (!bboxIntersect.has_value()) continue;  // no overlap at all

            const double intersectRatio =
                bboxIntersect->volume() /
                (0.5 * rootBbox.volume() + 0.5 * relativeBBox.volume());

            if (intersectRatio <
                params_.min_volume_intersection_ratio_for_lc_candidate)
                continue;

            const auto min_id = std::min<submap_id_t>(root_id, submapId);
            const auto max_id = std::max<submap_id_t>(root_id, submapId);

            PotentialLoop lc;
            lc.largest_id           = max_id;
            lc.smallest_id          = min_id;
            lc.topological_distance = ips.depth;
            lc.score                = intersectRatio;
            if (root_id == min_id)
                lc.relative_pose_largest_wrt_smallest = ips.pose;
            else
            {  // inverse SE(3) relative pose
                ips.pose.inverse(lc.relative_pose_largest_wrt_smallest);
            }

            potentialLCs.emplace(intersectRatio, lc);
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
            << " topo_depth=" << lc.topological_distance
            << " relPose: " << lc.relative_pose_largest_wrt_smallest.mean);
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
    const auto& mapGlobal    = submapGlobal.local_map;
    ASSERT_(mapGlobal);
    const auto& pcs_global = *mapGlobal;

    const auto  idLocal     = lc.largest_id;
    const auto& submapLocal = state_.submaps.at(idLocal);
    const auto& mapLocal    = submapLocal.local_map;
    ASSERT_(mapLocal);
    const auto& pcs_local = *mapLocal;

    MRPT_LOG_DEBUG_STREAM(
        "LC candidate: relPose=" << lc.relative_pose_largest_wrt_smallest);

    const mrpt::math::TPose3D initGuess =
        lc.relative_pose_largest_wrt_smallest.mean.asTPose();

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
    else
    {
        // do not re-localize, just start with the first initial guess:
        initGuesses.push_back(initGuess);
    }

    // 2) refine with ICP:
    // Goal: add as many graph edges as good ICP results, even if it seems
    // redundant. the robust kernels may help later on to tell which is the
    // correct one.

    bool atLeastOneGoodIcp = false;

    for (const auto& initPose : initGuesses)
    {
        mp2p_icp::Results icp_result;

        // Assume we are running this part in single thread (!)
        const size_t threadIdx = 0;

        auto& pts = state_.perThreadState_.at(threadIdx);

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

        // add a new graph edge:
        mrpt::poses::CPose3DPDFGaussian newEdge;
        newEdge.copyFrom(icp_result.optimal_tf);
        newEdge.cov.setIdentity();  // Not used anyway

        state_.submapsGraph.insertEdge(idGlobal, idLocal, newEdge);

        // and to the low-level graph too:
        const gtsam::Pose3 deltaPose =
            mrpt::gtsam_wrappers::toPose3(icp_result.optimal_tf.mean);

        using gtsam::symbol_shorthand::X;

        const double edge_std_xyz = 1.0;
        const double edge_std_ang = mrpt::DEG2RAD(5.0);

        const double icp_edge_robust_param = 1.0;

        gtsam::Vector6 sigmas;
        sigmas << edge_std_ang, edge_std_ang, edge_std_ang,  //
            edge_std_xyz, edge_std_xyz, edge_std_xyz;

        auto icpNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

        gtsam::noiseModel::Base::shared_ptr icpRobNoise;

        if (icp_edge_robust_param > 0)
            icpRobNoise = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                    icp_edge_robust_param),
                icpNoise);
        else
            icpRobNoise = icpNoise;

        state_.kfGraphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(*submapGlobal.kf_ids.begin()), X(*submapLocal.kf_ids.begin()),
            deltaPose, icpRobNoise);

        atLeastOneGoodIcp = true;
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
