/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mola_georeferencing/simplemap_georeference.h>
#include <mola_gtsam_factors/FactorGnssEnu.h>
#include <mola_sm_loop_closure/common/gnss_factor_helpers.h>
#include <mrpt/poses/gtsam_wrappers.h>

std::size_t mola::lc_common::add_gnss_factors_per_kf(
    gtsam::NonlinearFactorGraph& fg, const mrpt::maps::CSimpleMap& sm,
    std::optional<mrpt::topography::TGeodeticCoords>& geoRef, const GnssFactorParams& params,
    std::vector<uint64_t>& knownInliers, mrpt::system::COutputLogger* logger)
{
    using gtsam::symbol_shorthand::X;

    const auto gnssFrames = extract_gnss_frames_from_sm(sm, geoRef);

    if (gnssFrames.frames.empty())
    {
        if (logger)
        {
            logger->logFmt(
                mrpt::system::LVL_WARN, "[gnss_factor_helpers] No valid GNSS observations found");
        }
        return 0;
    }

    if (!geoRef.has_value())
    {
        geoRef = gnssFrames.refCoord;
    }

    // Noise model for optional horizontality prior: tight on roll/pitch, free on yaw.
    // GTSAM Pose3 tangent order: (Rx, Ry, Rz, tx, ty, tz)
    auto horizNoise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << params.horizontality_sigma_rpy,
                                             params.horizontality_sigma_rpy, 1e3, 1e3, 1e3, 1e3)
                                                .finished());

    std::size_t added    = 0;
    std::size_t rejected = 0;
    for (const auto& gf : gnssFrames.frames)
    {
        const auto frameId = static_cast<size_t>(gf.kf_index);
        if (frameId >= sm.size())
        {
            continue;
        }

        // Reject readings with excessive uncertainty
        const double horizUncertainty =
            std::sqrt(gf.sigma_E * gf.sigma_E + gf.sigma_N * gf.sigma_N);
        if (horizUncertainty > params.max_uncertainty_horiz ||
            gf.sigma_U > params.max_uncertainty_vert)
        {
            rejected++;
            continue;
        }

        const double sigE =
            std::max(gf.sigma_E * params.uncertainty_multiplier, params.minimum_uncertainty_xyz);
        const double sigN =
            std::max(gf.sigma_N * params.uncertainty_multiplier, params.minimum_uncertainty_xyz);
        const double sigU =
            std::max(gf.sigma_U * params.uncertainty_multiplier, params.minimum_uncertainty_xyz);

        auto noiseOrg    = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(sigE, sigN, sigU));
        auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.5), noiseOrg);

        const auto observedENU = mrpt::gtsam_wrappers::toPoint3(gf.enu);
        const auto sensorPointOnVeh =
            mrpt::gtsam_wrappers::toPoint3(gf.obs->sensorPose.translation());

        knownInliers.push_back(static_cast<uint64_t>(fg.size()));
        fg.emplace_shared<mola::factors::FactorGnssEnu>(
            X(frameId), sensorPointOnVeh, observedENU, robustNoise);
        added++;

        if (params.add_horizontality)
        {
            knownInliers.push_back(static_cast<uint64_t>(fg.size()));
            fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                X(frameId), gtsam::Pose3::Identity(), horizNoise);
            added++;
        }
    }

    if (logger)
    {
        const std::size_t total = gnssFrames.frames.size();
        logger->logFmt(
            mrpt::system::LVL_INFO,
            "[gnss_factor_helpers] GNSS readings: %zu accepted, %zu rejected (horiz>%.1fm or "
            "vert>%.1fm), %zu total. Added %zu factors.",
            total - rejected, rejected, params.max_uncertainty_horiz, params.max_uncertainty_vert,
            total, added);
    }

    return added;
}
