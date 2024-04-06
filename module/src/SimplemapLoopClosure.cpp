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

#include <mola_sm_loop_closure/SimplemapLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/Lie/SO.h>

// MRPT graph-slam:
#include <mrpt/graphs/dijkstra.h>
#include <mrpt/graphslam/levmarq.h>

// visualization:
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/graph_tools.h>

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

    // Obs2map merge pipeline:
    ASSERT_(c["insert_observation_into_local_map"].isSequence());
    // Create, and copy my own verbosity level:
    state_.obs2map_merge = mp2p_icp_filters::filter_pipeline_from_yaml(
        c["insert_observation_into_local_map"], this->getMinLoggingLevel());

    // Attach to the parameter source for dynamic parameters:
    mp2p_icp::AttachToParameterSource(
        state_.obs2map_merge, state_.parameter_source);

    ASSERT_(!state_.obs2map_merge.empty());

    if (cfg.has("gnns_sensor_label"))
        params_.gnns_sensor_label = cfg["gnns_sensor_label"].as<std::string>();

    YAML_LOAD_REQ(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, profiler_enabled, bool);
    YAML_LOAD_REQ(params_, submap_keyframe_count, size_t);

    YAML_LOAD_REQ(params_, threshold_sigma, double);
    YAML_LOAD_REQ(params_, max_sensor_range, double);

    YAML_LOAD_OPT(
        params_, min_volume_intersection_ratio_for_lc_candidate, double);

    ENSURE_YAML_ENTRY_EXISTS(c, "icp_settings");

    {
        const auto [icp, icpParams] =
            mp2p_icp::icp_pipeline_from_yaml(c["icp_settings"]);

        state_.icp             = icp;
        params_.icp_parameters = icpParams;
    }

    // Attach all ICP instances to the parameter source for dynamic
    // parameters:
    state_.icp->attachToParameterSource(state_.parameter_source);

    // system-wide profiler:
    profiler_.enable(params_.profiler_enabled);

    // Create lidar segmentation algorithm:
    {
        mrpt::system::CTimeLoggerEntry tle(
            profiler_, "filterPointCloud_initialize");

        // Observation -> map generator:
        if (c.has("observations_generator") &&
            !c["observations_generator"].isNullNode())
        {
            // Create, and copy my own verbosity level:
            state_.obs_generators = mp2p_icp_filters::generators_from_yaml(
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
            state_.obs_generators.push_back(defaultGen);
        }

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            state_.obs_generators, state_.parameter_source);

        if (c.has("observations_filter"))
        {
            // Create, and copy my own verbosity level:
            state_.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(
                c["observations_filter"], this->getMinLoggingLevel());

            // Attach to the parameter source for dynamic parameters:
            mp2p_icp::AttachToParameterSource(
                state_.pc_filter, state_.parameter_source);
        }

        // Local map generator:
        if (c.has("localmap_generator") &&
            !c["localmap_generator"].isNullNode())
        {
            // Create, and copy my own verbosity level:
            state_.local_map_generators =
                mp2p_icp_filters::generators_from_yaml(
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
            state_.local_map_generators, state_.parameter_source);

        // submaps final stage filter:
        state_.submap_final_filter =
            mp2p_icp_filters::filter_pipeline_from_yaml(
                c["submap_final_filter"], this->getMinLoggingLevel());

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            state_.submap_final_filter, state_.parameter_source);
    }

    state_.initialized = true;

    MRPT_TRY_END
}

// Find and apply loop closures in the input/output simplemap
void SimplemapLoopClosure::process(mrpt::maps::CSimpleMap& sm)
{
    ASSERT_(state_.initialized);

    state_.sm = &sm;

    // Minimum to have a large-enough topological loop closure:
    ASSERT_GT_(sm.size(), 3 * params_.submap_keyframe_count);

    // Build submaps:
    {
        std::set<keyframe_id_t> pendingKFs;
        for (size_t i = 0; i < sm.size(); i++)
        {
            pendingKFs.insert(i);

            if (pendingKFs.size() >= params_.submap_keyframe_count)
            {
                build_submap_from_kfs(pendingKFs);
                pendingKFs.clear();
            }
        }
        // remaining ones?
        if (!pendingKFs.empty()) build_submap_from_kfs(pendingKFs);
    }

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

                // TODO: extract cov from simplemap cov from icp odometry
                relPose.cov.setDiagonal(0.10);

                mrpt::poses::CPose3DPDFGaussianInf relPoseInf;
                relPoseInf.copyFrom(relPose);
                relPoseInf.cov_inv.setIdentity();  // XXX

                state_.submapsGraph.insertEdge(last_id, this_id, relPoseInf);
            }

            lastSubmap = &submap;
        }
    }

#if 1
    // Save viz of initial state:
    {
        VizOptions opts;
        auto       glMap = build_submaps_visualization(opts);

        mrpt::opengl::Scene scene;
        scene.insert(glMap);

        scene.saveToFile("submaps_initial.3Dscene");
    }
#endif

    // Look for potential loop closures:
    // -----------------------------------------------
    // find next smallest potential loop closure?

    std::set<std::pair<submap_id_t, submap_id_t>> alreadyChecked;

    // repeat until checkedCount==0:
    for (;;)
    {
        size_t checkedCount   = 0;
        bool   anyGraphChange = false;

        const PotentialLoopOutput& LCs = find_next_loop_closures();

        // check them all, and accept those that seem valid:
        for (const auto& [id0, lcs] : LCs)
        {
            for (const auto& lc : lcs)
            {
                auto IDs = std::make_pair(lc.smallest_id, lc.largest_id);
                ASSERT_EQUAL_(id0, IDs.first);

                if (alreadyChecked.count(IDs) != 0) continue;
                // a new pair. add it:
                alreadyChecked.insert(IDs);
                checkedCount++;

                MRPT_LOG_INFO_STREAM(
                    "Considering potential LC: " << lc.smallest_id << "<=>"
                                                 << lc.largest_id);

                const bool accepted = process_loop_candidate(lc);
                if (accepted) anyGraphChange = true;
            }
        }
        if (!checkedCount) break;  // no new LC was checked, we are done.

        // any change to the graph? re-optimize it:
        if (anyGraphChange)
        {
            const double rmseInit = state_.submapsGraph.chi2();

            mrpt::graphslam::TResultInfoSpaLevMarq lmOut;
            mrpt::graphslam::optimize_graph_spa_levmarq(
                state_.submapsGraph, lmOut);

            const double rmseFinal = state_.submapsGraph.chi2();

            MRPT_LOG_INFO_STREAM(
                "Graph re-optimized in " << lmOut.num_iters << " iters, RMSE="
                                         << rmseInit << " ==> " << rmseFinal);

            // Update submaps global pose:
            for (const auto& [id, pose] : state_.submapsGraph.nodes)
                state_.submaps.at(id).global_pose = pose;
        }
    }

#if 1
    // Save viz of initial state:
    {
        VizOptions opts;
        auto       glMap = build_submaps_visualization(opts);

        mrpt::opengl::Scene scene;
        scene.insert(glMap);

        scene.saveToFile("submaps_final.3Dscene");
    }
#endif

    // At this point, we have optimized the top-level submaps graph.
    // Now, update all low-level keyframes in the simplemap:
    mrpt::maps::CSimpleMap outSM;

    for (const auto& [submapId, submap] : state_.submaps)
    {
        const keyframe_id_t refKfId = *submap.kf_ids.begin();

        for (const auto kfId : submap.kf_ids)
        {
            const auto relPose =
                keyframe_relative_pose_in_simplemap(kfId, refKfId);
            const mrpt::poses::CPose3D newKfGlobalPose =
                submap.global_pose + relPose;

            auto& [oldPose, sf, twist] = state_.sm->get(kfId);

            const auto newPose = mrpt::poses::CPose3DPDFGaussian::Create();
            newPose->mean      = newKfGlobalPose;
            newPose->cov.setIdentity();  // TODO! Get cov from optimizer?

            outSM.insert(newPose, sf, twist);
        }
    }

    // Overwrite with new SM:
    sm = std::move(outSM);
}

void SimplemapLoopClosure::build_submap_from_kfs(
    const std::set<keyframe_id_t>& ids)
{
    ASSERT_(!ids.empty());

    const submap_id_t newSubmapId = state_.submaps.size();

    SubMap& submap = state_.submaps[newSubmapId];

    const keyframe_id_t refFrameId = *ids.begin();

    submap.id          = newSubmapId;
    submap.kf_ids      = ids;
    submap.global_pose = keyframe_pose_in_simplemap(refFrameId);

    MRPT_LOG_INFO_STREAM(
        "Populating submap #" << newSubmapId << " with " << ids.size()
                              << " keyframes...");

    // Insert all observations in this submap:
    for (const auto& id : ids)
    {
        // (this defines the local robot pose on the submap)
        updatePipelineDynamicVariablesForKeyframe(id, refFrameId);

        // now that we have valid dynamic variables, check if we need to create
        // the submap on the first KF:
        if (!submap.local_map)
        {
            // Create empty local map:
            submap.local_map = mp2p_icp::metric_map_t::Create();

            // and populate with empty metric maps:
            state_.parameter_source.realize();

            mrpt::obs::CSensoryFrame dummySF;
            {
                auto obs = mrpt::obs::CObservationPointCloud::Create();
                dummySF.insert(obs);
            }

            mp2p_icp_filters::apply_generators(
                state_.local_map_generators, dummySF, *submap.local_map);
        }

        // insert observations from the keyframe:
        // -------------------------------------------

        // Extract points from observation:
        auto observation = mp2p_icp::metric_map_t::Create();

        const auto& [pose, sf, twist] = state_.sm->get(id);

        mrpt::system::CTimeLoggerEntry tle0(
            profiler_, "add_submap_from_kfs.apply_generators");

        for (const auto& o : *sf)
            mp2p_icp_filters::apply_generators(
                state_.obs_generators, *o, *observation);

        tle0.stop();

        // Filter/segment the point cloud (optional, but normally will be
        // present):
        mrpt::system::CTimeLoggerEntry tle1(
            profiler_, "add_submap_from_kfs.filter_pointclouds");

        mp2p_icp_filters::apply_filter_pipeline(
            state_.pc_filter, *observation, profiler_);

        tle1.stop();

        // Merge "observation_layers_to_merge_local_map" in local map:
        // ---------------------------------------------------------------
        mrpt::system::CTimeLoggerEntry tle3(
            profiler_, "add_submap_from_kfs.update_local_map");

        // Input  metric_map_t: observation
        // Output metric_map_t: state_.local_map

        // 1/4: temporarily make a (shallow) copy of the observation layers into
        // the local map:
        for (const auto& [lyName, lyMap] : observation->layers)
        {
            ASSERTMSG_(
                submap.local_map->layers.count(lyName) == 0,
                mrpt::format(
                    "Error: local map layer name '%s' collides with one of the "
                    "observation layers, please use different layer names.",
                    lyName.c_str()));

            submap.local_map->layers[lyName] = lyMap;  // shallow copy
        }

        // 2/4: Make sure dynamic variables are up-to-date,
        // in particular, [robot_x, ..., robot_roll]
        // already done above: updatePipelineDynamicVariables();

        // 3/4: Apply pipeline
        mp2p_icp_filters::apply_filter_pipeline(
            state_.obs2map_merge, *submap.local_map, profiler_);

        // 4/4: remove temporary layers:
        for (const auto& [lyName, lyMap] : observation->layers)
            submap.local_map->layers.erase(lyName);

        tle3.stop();
    }  // end for each keyframe ID

    mp2p_icp_filters::apply_filter_pipeline(
        state_.submap_final_filter, *submap.local_map, profiler_);

    // add metadata:
    submap.local_map->id = newSubmapId;

    // Bbox: from point cloud layer:
    std::optional<mrpt::math::TBoundingBoxf> theBBox;

    for (const auto& [name, map] : submap.local_map->layers)
    {
        const auto* pts = mp2p_icp::MapToPointsMap(*map);
        if (!pts || pts->empty()) continue;

        auto bbox = pts->boundingBox();
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
    const keyframe_id_t id, const keyframe_id_t referenceId)
{
    const auto& [globalPose, sf, twist] = state_.sm->get(id);

    // Set dynamic variables for twist usage within ICP pipelines
    // (e.g. de-skew methods)
    {
        mrpt::math::TTwist3D twistForIcpVars = {0, 0, 0, 0, 0, 0};
        if (twist) twistForIcpVars = *twist;

        state_.parameter_source.updateVariable("VX", twistForIcpVars.vx);
        state_.parameter_source.updateVariable("VY", twistForIcpVars.vy);
        state_.parameter_source.updateVariable("VZ", twistForIcpVars.vz);
        state_.parameter_source.updateVariable("WX", twistForIcpVars.wx);
        state_.parameter_source.updateVariable("WY", twistForIcpVars.wy);
        state_.parameter_source.updateVariable("WZ", twistForIcpVars.wz);
    }

    // robot pose:
    const auto p = keyframe_relative_pose_in_simplemap(id, referenceId);

    state_.parameter_source.updateVariable("ROBOT_X", p.x());
    state_.parameter_source.updateVariable("ROBOT_Y", p.y());
    state_.parameter_source.updateVariable("ROBOT_Z", p.z());
    state_.parameter_source.updateVariable("ROBOT_YAW", p.yaw());
    state_.parameter_source.updateVariable("ROBOT_PITCH", p.pitch());
    state_.parameter_source.updateVariable("ROBOT_ROLL", p.roll());

    state_.parameter_source.updateVariable(
        "ADAPTIVE_THRESHOLD_SIGMA", params_.threshold_sigma);

    state_.parameter_source.updateVariable(
        "ESTIMATED_SENSOR_MAX_RANGE", params_.max_sensor_range);

    // Make all changes effective and evaluate the variables now:
    state_.parameter_source.realize();
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
        mrpt::poses::CPose3DPDFGaussianInf pose;
        size_t                             depth = 0;
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
            const mrpt::graphs::CNetworkOfPoses3DInf::edge_t*>;

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

    // filter them, and keep the most promising ones, sorted by "score",
    // and remove those touching a submap already included in another candidate:
    PotentialLoopOutput result;
    for (auto it = potentialLCs.rbegin(); it != potentialLCs.rend(); ++it)
    {
        const auto& [score, lc] = *it;

        // only store map entries for the "smallest_id" of each pair to avoid
        // duplicated checks:
        if (result.count(lc.largest_id) != 0) continue;

        auto& rs = result[lc.smallest_id];
        rs.emplace_back(lc);
    }

    // debug, print potential LCs:
    for (const auto& [id, lcs] : result)
    {
        for (const auto& lc : lcs)
        {
            MRPT_LOG_INFO_STREAM(
                "(2/2) Potential LC: "  //
                << id << " <==> " << lc.largest_id
                << " topo_depth=" << lc.topological_distance);
        }
    }

    return result;
}

bool SimplemapLoopClosure::process_loop_candidate(const PotentialLoop& lc)
{
    using namespace std::string_literals;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "process_loop_candidate");

    // Apply ICP between the two submaps:
    const auto  idGlobal  = lc.smallest_id;
    const auto& mapGlobal = state_.submaps.at(idGlobal).local_map;
    ASSERT_(mapGlobal);
    const auto& pcs_global = *mapGlobal;

    const auto  idLocal  = lc.largest_id;
    const auto& mapLocal = state_.submaps.at(idLocal).local_map;
    ASSERT_(mapLocal);
    const auto& pcs_local = *mapLocal;

    MRPT_LOG_DEBUG_STREAM(
        "LC candidate: relPose=" << lc.relative_pose_largest_wrt_smallest);

    mrpt::math::TPose3D current_solution =
        lc.relative_pose_largest_wrt_smallest.mean.asTPose();

    mp2p_icp::Results icp_result;

    state_.icp->align(
        pcs_local, pcs_global, current_solution, params_.icp_parameters,
        icp_result);

    MRPT_LOG_INFO_FMT(
        "ICP: goodness=%.02f%% iters=%u pose=%s "
        "termReason=%s",
        100.0 * icp_result.quality,
        static_cast<unsigned int>(icp_result.nIterations),
        icp_result.optimal_tf.getMeanVal().asString().c_str(),
        mrpt::typemeta::enum2str(icp_result.terminationReason).c_str());

    if (icp_result.quality < params_.min_icp_goodness) return false;

    // add a new graph edge:
    mrpt::poses::CPose3DPDFGaussianInf newEdge;  // to Information matrix
    newEdge.copyFrom(icp_result.optimal_tf);
    newEdge.cov_inv.setIdentity();  // XXX

    state_.submapsGraph.insertEdge(idGlobal, idLocal, newEdge);

    return true;
}
