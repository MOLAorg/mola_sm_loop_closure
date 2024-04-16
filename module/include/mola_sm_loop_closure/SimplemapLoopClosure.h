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

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

#include <regex>
#include <set>

namespace mola
{
/** LIDAR-inertial odometry based on ICP against a local metric map model.
 */
class SimplemapLoopClosure : public mrpt::system::COutputLogger
{
   public:
    SimplemapLoopClosure();
    ~SimplemapLoopClosure();

    /** @name Main API
     * @{ */

    using keyframe_id_t = uint32_t;
    using submap_id_t   = uint32_t;

    // See docs in base class
    void initialize(const mrpt::containers::yaml& cfg);

    /** Find and apply loop closures in the input/output simplemap */
    void process(mrpt::maps::CSimpleMap& sm);

    struct Parameters
    {
        /** List of sensor labels or regex's to be matched to input observations
         *  to be used as raw lidar observations.
         */
        std::vector<std::regex> lidar_sensor_labels;

        /** Sensor labels or regex to be matched to input observations
         *  to be used as GNNS (GPS) observations.
         */
        std::optional<std::regex> gnns_sensor_label;

        mp2p_icp::Parameters icp_parameters;

        double threshold_sigma_initial = 5.0;
        double threshold_sigma_final   = 0.5;

        double max_sensor_range = 100.0;

        double submap_max_length = 75.0;

        double min_volume_intersection_ratio_for_lc_candidate = 0.6;

        double      min_icp_goodness          = 0.60;
        bool        profiler_enabled          = true;
        bool        do_first_gross_relocalize = true;
        std::string debug_files_prefix        = "sm_lc_";
    };

    /** Algorithm parameters */
    Parameters params_;

    /** @} */

   private:
    // Each of the submaps for loop-closure checking:
    struct SubMap
    {
        SubMap() = default;

        submap_id_t id = 0;

        /// Global SE(3) pose of the submap in the global frame:
        mrpt::poses::CPose3D global_pose;

        /// Local metric map in the frame of coordinates of the submap:
        mp2p_icp::metric_map_t::Ptr local_map;

        mrpt::math::TBoundingBox bbox;  // in the submap local frame

        /// IDs are indices from the simplemap:
        std::set<keyframe_id_t> kf_ids;
    };

    struct State
    {
        State()  = default;
        ~State() = default;

        bool initialized = false;

        // Input SM being processed. To avoid passing it as parameter to all
        // methods.
        const mrpt::maps::CSimpleMap* sm = nullptr;

        struct PerThreadState
        {
            std::mutex mtx;

            mp2p_icp::ParameterSource parameter_source;

            mp2p_icp::ICP::Ptr icp;

            // observations:
            mp2p_icp_filters::GeneratorSet   obs_generators;
            mp2p_icp_filters::FilterPipeline pc_filter;

            // local maps:
            mp2p_icp_filters::GeneratorSet   local_map_generators;
            mp2p_icp_filters::FilterPipeline obs2map_merge;

            // final stage filters for submaps:
            mp2p_icp_filters::FilterPipeline submap_final_filter;
        };

        // One copy of the state per working thread:
        std::vector<PerThreadState> perThreadState_{
            std::thread::hardware_concurrency()};

        // Submaps:
        std::map<submap_id_t, SubMap> submaps;

        // This graph is used for Dijsktra only:
        mrpt::graphs::CNetworkOfPoses3DCov submapsGraph;

        // mrpt::graphs::CNetworkOfPoses3DInf keyframesGraph;

        gtsam::Values               kfGraphValues;
        gtsam::NonlinearFactorGraph kfGraphFG;

        mrpt::poses::CPose3D kfGraph_get_pose(const keyframe_id_t id) const;
    };

    State state_;

    mrpt::system::CTimeLogger profiler_{true, "sm_loop_closure"};

    // private methods:
    void build_submap_from_kfs_into(
        const std::set<keyframe_id_t>& ids, SubMap& submap,
        const size_t threadIdx);

    struct VizOptions
    {
        VizOptions() = default;

        std::string viz_point_layer = "minimap_viz";
        bool        show_bbox       = true;
        bool        show_edges      = true;
    };

    mrpt::opengl::CSetOfObjects::Ptr build_submaps_visualization(
        const VizOptions& p) const;

    mrpt::poses::CPose3D keyframe_pose_in_simplemap(keyframe_id_t kfId) const;
    mrpt::poses::CPose3D keyframe_relative_pose_in_simplemap(
        keyframe_id_t kfId, keyframe_id_t referenceKfId) const;

    void updatePipelineDynamicVariablesForKeyframe(
        const keyframe_id_t id, const keyframe_id_t referenceId,
        const size_t threadIdx);

    struct PotentialLoop
    {
        PotentialLoop() = default;

        submap_id_t                     smallest_id = 0, largest_id = 0;
        size_t                          topological_distance = 0;
        mrpt::poses::CPose3DPDFGaussian relative_pose_largest_wrt_smallest;
        double                          score = 0;
    };

    /** For each submap, a list of potential LCs to check, already sorted by
     * decreasing score */
    using PotentialLoopOutput = std::vector<PotentialLoop>;

    PotentialLoopOutput find_next_loop_closures() const;

    [[nodiscard]] bool process_loop_candidate(const PotentialLoop& lc);
};

}  // namespace mola
