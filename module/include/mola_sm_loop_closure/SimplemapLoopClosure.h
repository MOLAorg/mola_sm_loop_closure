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

#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

#include <regex>

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
         *  to be used as raw IMU observations.
         */
        std::optional<std::regex> imu_sensor_label;

        /** Sensor labels or regex to be matched to input observations
         *  to be used as wheel odometry observations.
         */
        std::optional<std::regex> wheel_odometry_sensor_label;

        /** Sensor labels or regex to be matched to input observations
         *  to be used as GNNS (GPS) observations.
         */
        std::optional<std::regex> gnns_sensor_label;

        struct ICP_case
        {
            mp2p_icp::ICP::Ptr   icp;
            mp2p_icp::Parameters icp_parameters;
        };

        ICP_case icp;

        double min_icp_goodness = 0.60;
        bool   profiler_enabled = true;
    };

    /** Algorithm parameters */
    Parameters params_;

    /** @} */

   private:
    struct ICP_Input
    {
        using Ptr = std::shared_ptr<ICP_Input>;

        int32_t                     global_id = -1;
        int32_t                     local_id  = -1;
        mp2p_icp::metric_map_t::Ptr global_pc, local_pc;
        mrpt::math::TPose3D         init_guess_local_wrt_global;
        mp2p_icp::Parameters        icp_params;

        std::optional<mrpt::poses::CPose3DPDFGaussianInf> prior;

        /** used to identity where does this request come from */
        std::string debug_str;
    };
    struct ICP_Output
    {
        double                          goodness{.0};
        mrpt::poses::CPose3DPDFGaussian found_pose_to_wrt_from;
    };
    void run_one_icp(const ICP_Input& in, ICP_Output& out);

    struct State
    {
        State()  = default;
        ~State() = default;

        bool initialized = false;

        mp2p_icp::ParameterSource parameter_source;

        // observations:
        mp2p_icp_filters::GeneratorSet   obs_generators;
        mp2p_icp_filters::FilterPipeline pc_filter;

        // local maps:
        mp2p_icp_filters::GeneratorSet   local_map_generators;
        mp2p_icp_filters::FilterPipeline obs2map_merge;
    };

    State state_;

    mrpt::system::CTimeLogger profiler_{true, "sm_loop_closure"};
};

}  // namespace mola
