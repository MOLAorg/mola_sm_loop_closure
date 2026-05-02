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

#pragma once

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <mola_sm_loop_closure/LoopClosureInterface.h>
#include <mola_sm_loop_closure/common/icp_pipeline_setup.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/system/CTimeLogger.h>

#include <future>
#include <set>

namespace mola
{
/** LIDAR-inertial loop closure engine.
 */
class SimplemapLoopClosure : public mola::LoopClosureInterface
{
    DEFINE_MRPT_OBJECT(SimplemapLoopClosure, mola)

   public:
    SimplemapLoopClosure();

    /** @name Main API
     * @{ */

    using keyframe_id_t = uint32_t;
    using submap_id_t   = uint32_t;

    // See docs in base class
    void initialize(const mrpt::containers::yaml& cfg) override;

    /** Find and apply loop closures in the input/output simplemap */
    void process(mrpt::maps::CSimpleMap& sm) override;

    struct Parameters
    {
        mp2p_icp::Parameters icp_parameters;

        std::string           threshold_sigma_initial                        = "5.0";
        std::string           threshold_sigma_final                          = "0.5";
        double                max_sensor_range                               = 100.0;
        double                icp_edge_additional_noise_xyz                  = 1e-2;
        double                icp_edge_additional_noise_ang_deg              = 0.05;
        double                input_odometry_edge_additional_noise_xyz       = 0.001;
        double                input_odometry_edge_additional_noise_ang_deg   = 0.001;
        double                input_edges_uncertainty_multiplier             = 1.0;
        double                submap_max_length_wrt_map                      = 0.10;
        double                submap_max_absolute_length                     = 100.0;
        double                submap_min_absolute_length                     = 50.0;
        double                max_time_between_kfs_to_break_submap           = 10.0;
        double                min_volume_intersection_ratio_for_lc_candidate = 0.6;
        bool                  assume_planar_world                            = false;
        double                planar_world_initial_sigma_z                   = 0.10;  // [m]
        double                planar_world_initial_sigma_ang                 = 0.02;  // [rad]
        size_t                planar_world_annealing_rounds                  = 2;
        bool                  planar_world_hard_flatten                      = false;
        std::array<double, 6> submap_graph_edge_sigmas                       = {
                                  0.10, 0.10, 0.15, 1.0 * M_PI / 180, 1.5 * M_PI / 180, 1.0 * M_PI / 180};
        bool use_gnss = true;
        /// "submap" (default, scalable for very large maps) or "per_kf"
        /// (sensor-pose aware, more accurate, larger factor graph).
        std::string gnss_factor_strategy         = "submap";
        bool        gnss_add_horizontality       = false;
        double      gnss_horizontality_sigma_rpy = 0.01;  // [rad]
        double      gnss_minimum_uncertainty_xyz = 0.10;  // [m]
        double      gnss_max_uncertainty_horiz = 20.0;  // [m] reject if sqrt(sigE²+sigN²) > this
        double      gnss_max_uncertainty_vert  = 40.0;  // [m] reject if sigU > this
        bool        use_imu_gravity            = true;
        double      imu_gravity_sigma_deg      = 3.0;  // [deg]
        uint32_t    max_number_lc_candidates   = 150;  // 0: no limit
        double      min_lc_uncertainty_ratio_to_draw_several_samples = 2.0;
        double      largest_delta_for_reconsider_all                 = 10.0;
        uint32_t    max_number_lc_candidates_per_submap              = 4;
        double      min_icp_goodness                                 = 0.60;
        bool        profiler_enabled                                 = true;
        bool        do_first_gross_relocalize                        = false;
        bool        do_montecarlo_icp                                = false;
        std::string debug_files_prefix                               = "sm_lc_";
        bool        save_submaps_viz_files                           = true;
    };

    /** Algorithm parameters */
    Parameters params_;

    /** @} */

   private:
    // Each of the submaps for loop-closure checking:
    struct SubMap
    {
        SubMap() = default;

        /// Global SE(3) pose of the submap in the global frame:
        mrpt::poses::CPose3D global_pose;

        /// Local metric map in the frame of coordinates of the submap:
        mutable mp2p_icp::metric_map_t::Ptr local_map;

        submap_id_t id = 0;

        mutable mrpt::math::TBoundingBox bbox;  // in the submap local frame
        mutable bool                     bbox_ready = false;

        // Pending async local-map build (used when bbox_ready == false).
        mutable std::shared_future<mp2p_icp::metric_map_t::Ptr> pending_local_map_fut;

        std::optional<mp2p_icp::metric_map_t::Georeferencing> geo_ref;

        /// IDs are indices from the simplemap:
        std::set<keyframe_id_t> kf_ids;
    };

    /** Get (or build upon first request) the metric local map of a submap
     */
    std::future<mp2p_icp::metric_map_t::Ptr> get_submap_local_map(const SubMap& submap);

    mp2p_icp::metric_map_t::Ptr impl_get_submap_local_map(const SubMap& submap);

    /** Ensure submap bbox is populated; blocks if a deferred build is pending.
     */
    void ensure_submap_bbox_ready(const SubMap& submap) const;

    struct State
    {
        State() = default;

        bool initialized = false;

        // Input SM being processed. To avoid passing it as parameter to all
        // methods.
        const mrpt::maps::CSimpleMap* sm = nullptr;

        struct PerThreadState
        {
            std::mutex mtx;

            // Shared ICP pipeline (obs generators, filter, parameter source):
            lc_common::PerThreadIcpPipeline pipeline;

            // SM-specific pipelines:
            mp2p_icp_filters::GeneratorSet   local_map_generators;
            mp2p_icp_filters::FilterPipeline obs2map_merge;
            mp2p_icp_filters::FilterPipeline submap_final_filter;

            double REL_POSE_SIGMA_XY = 1.0;
        };

        // One copy of the state per working thread:
        std::vector<PerThreadState> perThreadState_{
            std::max(1u, std::thread::hardware_concurrency())};

        // Submaps:
        std::map<submap_id_t, SubMap> submaps;

        // This graph is used for Dijkstra only:
        mrpt::graphs::CNetworkOfPoses3DCov submapsGraph;

        std::optional<mrpt::topography::TGeodeticCoords> globalGeoRef;
        submap_id_t                                      globalGeoRefSubmapId{};

        gtsam::Values                   kfGraphValues;
        gtsam::NonlinearFactorGraph     kfGraphFG;
        gtsam::NonlinearFactorGraph     planarityFG;
        std::vector<uint64_t>           knownInlierFactorIndices;
        std::optional<gtsam::Marginals> kfGraphMarginals;

        mrpt::poses::CPose3D kfGraph_get_pose(keyframe_id_t id) const;
    };

    State state_;

    mrpt::system::CTimeLogger profiler_{true, "sm_loop_closure"};

    // private methods:
    void build_submap_from_kfs_into(const std::set<keyframe_id_t>& ids, SubMap& submap);

    struct VizOptions
    {
        VizOptions() = default;

        std::string viz_point_layer = "minimap_viz";
        bool        show_bbox       = true;
        bool        show_edges      = true;
    };

    mrpt::opengl::CSetOfObjects::Ptr build_submaps_visualization(const VizOptions& p) const;

    mrpt::poses::CPose3D keyframe_pose_in_simplemap(keyframe_id_t kfId) const;
    mrpt::poses::CPose3D keyframe_relative_pose_in_simplemap(
        keyframe_id_t kfId, keyframe_id_t referenceKfId) const;

    void updatePipelineDynamicVariablesForKeyframe(
        keyframe_id_t id, keyframe_id_t referenceId, size_t threadIdx);

    struct PotentialLoop
    {
        PotentialLoop() = default;

        mrpt::poses::CPose3DPDFGaussian relative_pose_largest_wrt_smallest;
        bool                            draw_several_samples = false;
        size_t                          topological_distance = 0;
        double                          score                = 0;
        submap_id_t                     smallest_id = 0, largest_id = 0;
    };

    /** For each submap, a list of potential LCs to check, already sorted by
     * decreasing score */
    using PotentialLoopOutput = std::vector<PotentialLoop>;

    PotentialLoopOutput find_next_loop_closures(
        const std::set<std::pair<submap_id_t, submap_id_t>>& alreadyChecked) const;

    [[nodiscard]] bool process_loop_candidate(const PotentialLoop& lc);

    mrpt::WorkerThreadsPool threads_{state_.perThreadState_.size()};

    /// Optimizes the graph and returns the largestDelta
    double optimize_graph();

    /// Detect submaps in "state_.sm".
    std::vector<std::set<keyframe_id_t>> detect_sub_maps() const;

    void save_current_key_frame_poses_as_tum(const std::string& outTumFile) const;

    void add_gnss_factors_per_kf();
};

}  // namespace mola
