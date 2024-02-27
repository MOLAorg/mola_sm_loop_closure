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
#include <mrpt/poses/Lie/SO.h>

using namespace mola;

SimplemapLoopClosure::SimplemapLoopClosure()
{
    mrpt::system::COutputLogger::setLoggerName("SimplemapLoopClosure");
}

SimplemapLoopClosure::~SimplemapLoopClosure() = default;

namespace
{
void load_icp_set_of_params(
    SimplemapLoopClosure::Parameters::ICP_case& out,
    const mrpt::containers::yaml&               cfg)
{
    const auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(cfg);

    out.icp            = icp;
    out.icp_parameters = params;
}
}  // namespace

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

    if (cfg.has("imu_sensor_label"))
        params_.imu_sensor_label = cfg["imu_sensor_label"].as<std::string>();

    if (cfg.has("wheel_odometry_sensor_label"))
        params_.wheel_odometry_sensor_label =
            cfg["wheel_odometry_sensor_label"].as<std::string>();

    if (cfg.has("gnns_sensor_label"))
        params_.gnns_sensor_label = cfg["gnns_sensor_label"].as<std::string>();

    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, profiler_enabled, bool);

    ENSURE_YAML_ENTRY_EXISTS(c, "icp_settings");
    load_icp_set_of_params(params_.icp, c["icp_settings"]);

    // Attach all ICP instances to the parameter source for dynamic
    // parameters:
    params_.icp.icp->attachToParameterSource(state_.parameter_source);

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
            std::cout << "[warning] Using default mp2p_icp_filters::Generator "
                         "for the local map since no YAML 'localmap_generator' "
                         "entry was given\n";

            auto defaultGen = mp2p_icp_filters::Generator::Create();
            defaultGen->initialize({});
            state_.local_map_generators.push_back(defaultGen);
        }
        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            state_.local_map_generators, state_.parameter_source);
    }

    // Parameterizable values in params_:

    state_.initialized = true;

    MRPT_TRY_END
}

// TODO: Factor this class (also in mola_lo) into an independent pkg
std::tuple<bool /*isFirst*/, mrpt::poses::CPose3D /*distanceToClosest*/>
    SimplemapLoopClosure::SearchablePoseList::check(
        const mrpt::poses::CPose3D& p) const
{
    const bool           isFirst = empty();
    mrpt::poses::CPose3D distanceToClosest;
    if (isFirst) return {isFirst, distanceToClosest};

    if (from_last_only_)
    {  //
        distanceToClosest = p - last_kf_;
    }
    else
    {
        std::vector<mrpt::math::TPoint3Df> closest;
        std::vector<float>                 closestSqrDist;
        std::vector<uint64_t>              closestID;

        kf_points_.nn_multiple_search(
            p.translation().cast<float>(), 20, closest, closestSqrDist,
            closestID);
        ASSERT_(!closest.empty());  // empty()==false from check above

        // Check for both, rotation and translation.
        // Use a heuristic SE(3) metric to merge both parts:
        constexpr double ROTATION_WEIGHT = 1.0;

        std::optional<size_t> bestIdx;

        for (size_t i = 0; i < closest.size(); i++)
        {
            const auto&  candidate = kf_poses_.at(closestID.at(i));
            const double rot       = mrpt::poses::Lie::SO<3>::log(
                                   (p - candidate).getRotationMatrix())
                                   .norm();

            closestSqrDist[i] += ROTATION_WEIGHT * mrpt::square(rot);

            if (!bestIdx || closestSqrDist[i] < closestSqrDist[*bestIdx])
                bestIdx = i;
        }

        const auto& closestPose = kf_poses_.at(closestID.at(*bestIdx));

        distanceToClosest = p - closestPose;
    }

    return {isFirst, distanceToClosest};
}

void SimplemapLoopClosure::SearchablePoseList::removeAllFartherThan(
    const mrpt::poses::CPose3D& p, const double maxTranslation)
{
    if (from_last_only_) return;  // not applicable

    std::deque<mrpt::poses::CPose3D> new_kf_poses;
    mrpt::maps::CSimplePointsMap     new_kf_points;

    const double maxSqrDist = mrpt::square(maxTranslation);
    const auto   c          = p.translation();

    for (size_t i = 0; i < kf_poses_.size(); i++)
    {
        mrpt::math::TPoint3D pt;
        kf_points_.getPoint(i, pt.x, pt.y, pt.z);
        if ((pt - c).sqrNorm() > maxSqrDist) continue;  // remove
        // pass:
        new_kf_points.insertPoint(pt);
        new_kf_poses.push_back(kf_poses_.at(i));
    }
    // replace:
    kf_poses_  = std::move(new_kf_poses);
    kf_points_ = std::move(new_kf_points);
}

// Find and apply loop closures in the input/output simplemap
void SimplemapLoopClosure::process(mrpt::maps::CSimpleMap& sm)
{
    //
}
