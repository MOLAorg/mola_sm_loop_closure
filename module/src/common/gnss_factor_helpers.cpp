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

#include <algorithm>
#include <limits>

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

    // Track the spatial spread and uncertainty of the accepted observations so
    // we can detect a degenerate configuration after the loop.
    mrpt::math::TPoint3D bbMin;
    mrpt::math::TPoint3D bbMax;
    double               minSigma    = std::numeric_limits<double>::max();
    std::size_t          acceptedObs = 0;

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

        // Accumulate spread / uncertainty of accepted observations:
        if (acceptedObs == 0)
        {
            bbMin = gf.enu;
            bbMax = gf.enu;
        }
        else
        {
            bbMin.x = std::min(bbMin.x, gf.enu.x);
            bbMin.y = std::min(bbMin.y, gf.enu.y);
            bbMin.z = std::min(bbMin.z, gf.enu.z);
            bbMax.x = std::max(bbMax.x, gf.enu.x);
            bbMax.y = std::max(bbMax.y, gf.enu.y);
            bbMax.z = std::max(bbMax.z, gf.enu.z);
        }
        minSigma = std::min({minSigma, sigE, sigN, sigU});
        acceptedObs++;

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

    // Strong degeneracy check: the spatial spread of the accepted GNSS
    // observations must be large compared to their uncertainty, otherwise the
    // global-attitude estimation is ill-conditioned (the map roll/pitch becomes
    // unobservable and can take absurd values). We require the ENU bounding-box
    // diagonal to be > 3x the minimum per-axis sigma.
    if (acceptedObs >= 2)
    {
        const double bboxDiagonal = (bbMax - bbMin).norm();
        ASSERTMSG_(
            bboxDiagonal > 3.0 * minSigma,
            mrpt::format(
                "Degenerate GNSS configuration: the ENU bounding-box diagonal "
                "(%.3f m) of the %zu accepted GNSS observations is not larger "
                "than 3x their minimum per-axis sigma (3 * %.3f = %.3f m). The "
                "global map attitude (roll/pitch/yaw) is unobservable and would "
                "take absurd values. Provide GNSS observations with a larger "
                "spatial spread, or relax the gnss_max_uncertainty_* thresholds "
                "so that more (well-distributed) fixes are accepted.",
                bboxDiagonal, acceptedObs, minSigma, 3.0 * minSigma));
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
