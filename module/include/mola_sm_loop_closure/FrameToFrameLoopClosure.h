// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
// Licensed under the GNU GPL v3.
//
// This file is part of MOLA.
// MOLA is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// MOLA. If not, see <https://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/topography/data_types.h>

#include <set>
#include <vector>

namespace mola
{
/** Frame-to-frame GNSS-assisted loop closure engine.
 *
 * This class implements a simpler loop closure strategy than SimplemapLoopClosure:
 * 1) Uses GPS readings to optimize global frame poses
 * 2) Runs frame-to-frame ICP between loop closure candidates
 * 3) Optimizes the full graph with robust factors
 */
class FrameToFrameLoopClosure : public mrpt::system::COutputLogger
{
   public:
    FrameToFrameLoopClosure();

    /** @name Main API
     * @{ */

    using frame_id_t = uint32_t;

    void initialize(const mrpt::containers::yaml& cfg);

    /** Find and apply loop closures in the input/output simplemap */
    void process(mrpt::maps::CSimpleMap& sm);

    struct Parameters
    {
        mp2p_icp::Parameters icp_parameters;

        // GNSS optimization parameters
        bool   use_gnss                     = true;
        double gnss_minimum_uncertainty_xyz = 0.10;  // [m]
        bool   gnss_add_horizontality       = false;
        double gnss_horizontality_sigma_z   = 0.01;  // [m]

        // Loop closure candidate selection
        double min_distance_between_frames   = 20.0;  // [m] minimum separation for LC
        double max_distance_for_lc_candidate = 50.0;  // [m] maximum distance to consider
        size_t max_lc_candidates             = 100;  // maximum candidates to check
        size_t min_frames_between_lc         = 50;  // minimum frame separation

        // ICP parameters
        double      min_icp_goodness              = 0.50;
        double      icp_edge_robust_param         = 5.0;
        double      icp_edge_additional_noise_xyz = 0.02;  // [m]
        double      icp_edge_additional_noise_ang = 0.1;  // [deg]
        std::string threshold_sigma_initial       = "0.10";
        std::string threshold_sigma_final         = "0.05";

        // Odometry edge parameters
        double input_odometry_noise_xyz           = 0.01;  // [m]
        double input_odometry_noise_ang           = 0.1;  // [deg]
        double input_edges_uncertainty_multiplier = 1.0;

        // Optimization parameters
        double largest_delta_for_reconsider = 15.0;  // [m] re-check LCs if change > this

        // Sensor parameters
        double max_sensor_range = 100.0;  // [m]

        // Output and profiling
        bool        profiler_enabled      = true;
        bool        save_trajectory_files = true;
        std::string debug_files_prefix    = "f2f_lc_";
    };

    Parameters params_;

    /** @} */

   private:
    struct PerThreadState
    {
        std::mutex mtx;

        mp2p_icp::ParameterSource parameter_source;
        mp2p_icp::ICP::Ptr        icp;

        // For processing observations
        mp2p_icp_filters::GeneratorSet   obs_generators;
        mp2p_icp_filters::FilterPipeline pc_filter;

        mrpt::expr::CRuntimeCompiledExpression expr_threshold_sigma_initial;
        mrpt::expr::CRuntimeCompiledExpression expr_threshold_sigma_final;
    };

    struct State
    {
        bool initialized = false;

        const mrpt::maps::CSimpleMap* sm = nullptr;

        // Per-thread ICP instances
        std::vector<PerThreadState> perThreadState_{
            std::max(1u, std::thread::hardware_concurrency())};

        // GNSS reference coordinate
        std::optional<mrpt::topography::TGeodeticCoords> globalGeoRef;

        // GTSAM graph and values
        gtsam::Values               graphValues;
        gtsam::NonlinearFactorGraph graphFG;

        [[nodiscard]] mrpt::poses::CPose3D get_pose(frame_id_t id) const;
    };

    State state_;

    mrpt::system::CTimeLogger profiler_{true, "frame_to_frame_lc"};
    mrpt::WorkerThreadsPool   threads_{state_.perThreadState_.size()};

    // Private methods
    mrpt::poses::CPose3D frame_pose_in_simplemap(frame_id_t frameId) const;

    /** Generate point cloud from a frame's observations */
    mp2p_icp::metric_map_t::Ptr generate_frame_pointcloud(frame_id_t frameId, size_t threadIdx);

    /** Build initial graph with odometry and GNSS factors */
    void build_initial_graph();

    /** Add GNSS factors to the graph */
    void add_gnss_factors();

    struct LoopCandidate
    {
        frame_id_t frame_i  = 0;
        frame_id_t frame_j  = 0;
        double     distance = 0.0;  // estimated distance between frames
        double     score    = 0.0;  // candidate quality score
    };

    /** Find potential loop closure candidates */
    std::vector<LoopCandidate> find_loop_candidates(
        const std::set<std::pair<frame_id_t, frame_id_t>>& alreadyChecked) const;

    /** Process a single loop closure candidate with ICP */
    bool process_loop_candidate(const LoopCandidate& lc);

    /** Optimize the graph and return the largest pose change */
    double optimize_graph();

    /** Save trajectory to TUM format file */
    void save_trajectory_as_tum(const std::string& filename) const;

    /** Update dynamic variables for ICP pipeline */
    void update_dynamic_variables(frame_id_t frameId, size_t threadIdx);
};

}  // namespace mola