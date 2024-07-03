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

// MOLA+MRPT
#include <mola_sm_loop_closure/simplemap_georeference.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

// gtsam factors:
#include "FactorGNSS2ENU.h"

mola::SMGeoReferencingOutput mola::simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm, const SMGeoReferencingParams& params)
{
    mola::SMGeoReferencingOutput ret;

    ASSERT_(!sm.empty());

    const GNSSFrames smFrames =
        extract_gnss_frames_from_sm(sm, params.geodeticReference);

    // Build and optimize GTSAM graph:
    using gtsam::symbol_shorthand::P;  // P(i): each vehicle pose
    using gtsam::symbol_shorthand::T;  // T(0): the single sought transformation

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               v;

    add_gnss_factors(graph, v, smFrames, params.fgParams);

    gtsam::LevenbergMarquardtParams lmParams =
        gtsam::LevenbergMarquardtParams::CeresDefaults();

    gtsam::LevenbergMarquardtOptimizer lm(graph, v, lmParams);

    auto optimal = lm.optimize();

    const double errInit = graph.error(v);
    const double errEnd  = graph.error(optimal);

    const double rmseInit = std::sqrt(errInit / graph.size());
    const double rmseEnd  = std::sqrt(errEnd / graph.size());

    gtsam::Marginals marginals(graph, optimal);

    const auto T0     = optimal.at<gtsam::Pose3>(T(0));
    const auto T0_cov = marginals.marginalCovariance(T(0));
    const auto stds   = T0_cov.diagonal().array().sqrt().eval();

    if (params.logger)
    {
        std::stringstream ss;
        ss << "[simplemap_georeference] LM iterations: " << lm.iterations()
           << ", init error: " << errInit << " (rmse=" << rmseInit
           << "), final error: " << errEnd << "(rmse=" << rmseEnd << ") , for "
           << smFrames.frames.size() << " frames, sigmas: " << stds.transpose();
        params.logger->logStr(mrpt::system::LVL_INFO, ss.str());
    }

    // store results:
    ret.geo_ref.T_enu_to_map = {
        mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T0)),
        mrpt::gtsam_wrappers::to_mrpt_se3_cov6(T0_cov)};

    ret.geo_ref.geo_coord = *smFrames.refCoord;

    ret.final_rmse = rmseEnd;

    return ret;
}

mola::GNSSFrames mola::extract_gnss_frames_from_sm(
    const mrpt::maps::CSimpleMap&                           sm,
    const std::optional<mrpt::topography::TGeodeticCoords>& refCoordIn)
{
    GNSSFrames ret;

    ret.refCoord = refCoordIn;

    ret.frames.reserve(sm.size());

    // Build list of KF poses with GNSS observations:
    for (const auto& [pose, sf, twist] : sm)
    {
        ASSERT_(pose);
        ASSERT_(sf);

        const auto p = pose->getMeanVal();

        mrpt::obs::CObservationGPS::Ptr obs;
        for (size_t i = 0;
             !!(obs = sf->getObservationByClass<mrpt::obs::CObservationGPS>(i));
             i++)
        {
            if (!obs->hasMsgType(mrpt::obs::gnss::NMEA_GGA)) continue;

            auto& f = ret.frames.emplace_back();

            f.pose = p;
            f.obs  = obs;
            f.gga  = obs->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

            if (obs->covariance_enu)
            {
                f.sigma_E = std::sqrt((*obs->covariance_enu)(0, 0));
                f.sigma_N = std::sqrt((*obs->covariance_enu)(1, 1));
                f.sigma_U = std::sqrt((*obs->covariance_enu)(2, 2));
            }
            else
            {
                f.sigma_E = f.gga.fields.HDOP * 4.5 /*HDOP_REFERENCE_METERS*/;
                f.sigma_N = f.sigma_E;
                f.sigma_U = f.sigma_E;
            }

            ASSERT_(f.sigma_E > 0);
            ASSERT_(f.sigma_N > 0);
            ASSERT_(f.sigma_U > 0);

            f.coords.lat    = f.gga.fields.latitude_degrees;
            f.coords.lon    = f.gga.fields.longitude_degrees;
            f.coords.height = f.gga.fields.altitude_meters;

            // keep first one:
            if (!ret.refCoord.has_value()) ret.refCoord = f.coords;

            // Convert GNSS obs to ENU:
            mrpt::topography::geodeticToENU_WGS84(
                f.coords, f.enu, *ret.refCoord);
        }
    }

    return ret;
}

void mola::add_gnss_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const GNSSFrames& frames,
    const AddGNSSFactorParams& params)
{
    using gtsam::symbol_shorthand::P;  // P(i): each vehicle pose
    using gtsam::symbol_shorthand::T;  // T(0): the single sought transformation

    v.insert(T(0), gtsam::Pose3::Identity());

    // Expression to optimize (i=0...N):
    // P (+) kf_pose{i} = gps_enu{i}

    auto noisePoses         = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
    auto noiseHorizontality = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(1e3, 1e3, 1e3, 1e6, 1e6, params.horizontalitySigmaZ));

    for (size_t i = 0; i < frames.frames.size(); i++)
    {
        const auto& frame = frames.frames.at(i);

        auto noiseOrg = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(frame.sigma_E, frame.sigma_N, frame.sigma_U));

        auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.5), noiseOrg);

        const auto observedENU = mrpt::gtsam_wrappers::toPoint3(frame.enu);
        const auto sensorPointOnVeh =
            mrpt::gtsam_wrappers::toPoint3(frame.obs->sensorPose.translation());

        fg.emplace_shared<FactorGNSS2ENU>(
            P(i), sensorPointOnVeh, observedENU, robustNoise);

        const auto vehiclePose = mrpt::gtsam_wrappers::toPose3(frame.pose);

        v.insert(P(i), vehiclePose);

        fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            T(0), P(i), vehiclePose, noisePoses);

        if (params.addHorizontalityConstraints)
        {
            fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                P(i), gtsam::Pose3::Identity(), noiseHorizontality);
        }
    }
}
