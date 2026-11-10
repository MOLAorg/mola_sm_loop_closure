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
#include <mola_gtsam_factors/MapGravityFactor.h>
#include <mola_sm_loop_closure/common/gnss_factor_helpers.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
/// The uncertainty rules for a GNSS reading to be used, shared by the factors
/// and the initial alignment so both use the same readings.
bool gnss_reading_accepted(
    const mola::FrameGNSS& gf, const mola::lc_common::GnssFactorParams& params)
{
    const double horizUncertainty = std::sqrt(gf.sigma_E * gf.sigma_E + gf.sigma_N * gf.sigma_N);
    return horizUncertainty <= params.max_uncertainty_horiz &&
           gf.sigma_U <= params.max_uncertainty_vert;
}
}  // namespace

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

    // Noise model for optional horizontality constraint: the vehicle up-vector must
    // point along {enu} +Z, which leaves yaw free whatever its value. A prior on the
    // Pose3 rotation tangent would not: its x,y components mix with yaw.
    auto horizNoise = gtsam::noiseModel::Isotropic::Sigma(3, params.horizontality_sigma_rpy);

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
        if (!gnss_reading_accepted(gf, params))
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
            fg.emplace_shared<mola::factors::MapGravityFactor>(
                X(frameId), gtsam::Pose3::Identity(), gtsam::Vector3::UnitZ(), horizNoise);
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

std::optional<mrpt::poses::CPose3D> mola::lc_common::fit_yaw_translation(
    const std::vector<mrpt::math::TPoint3D>& src, const std::vector<mrpt::math::TPoint3D>& dst,
    double minHorizontalSpread)
{
    ASSERT_EQUAL_(src.size(), dst.size());
    const size_t n = src.size();
    if (n < 3)
    {
        return std::nullopt;
    }

    mrpt::math::TPoint3D srcMean(0, 0, 0);
    mrpt::math::TPoint3D dstMean(0, 0, 0);
    for (size_t i = 0; i < n; i++)
    {
        srcMean += src[i];
        dstMean += dst[i];
    }
    srcMean *= 1.0 / static_cast<double>(n);
    dstMean *= 1.0 / static_cast<double>(n);

    double sumCross  = 0;
    double sumDot    = 0;
    double sumSpread = 0;
    for (size_t i = 0; i < n; i++)
    {
        const double ax = src[i].x - srcMean.x;
        const double ay = src[i].y - srcMean.y;
        const double bx = dst[i].x - dstMean.x;
        const double by = dst[i].y - dstMean.y;
        sumCross += ax * by - ay * bx;
        sumDot += ax * bx + ay * by;
        sumSpread += ax * ax + ay * ay;
    }
    if (std::sqrt(sumSpread / static_cast<double>(n)) < minHorizontalSpread)
    {
        return std::nullopt;
    }

    const double yaw            = std::atan2(sumCross, sumDot);
    const auto   rot            = mrpt::poses::CPose3D::FromYawPitchRoll(yaw, 0, 0);
    const auto   rotatedSrcMean = rot.rotateVector(srcMean);
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        dstMean.x - rotatedSrcMean.x, dstMean.y - rotatedSrcMean.y, dstMean.z - rotatedSrcMean.z,
        yaw, 0, 0);
}

std::optional<mrpt::poses::CPose3D> mola::lc_common::fit_yaw_translation_to_gnss(
    const mrpt::maps::CSimpleMap& sm, std::optional<mrpt::topography::TGeodeticCoords>& geoRef,
    const GnssFactorParams& params, const std::function<mrpt::poses::CPose3D(size_t)>& kfPose,
    mrpt::system::COutputLogger* logger)
{
    // Below this spread, the yaw of the fit is not trustworthy:
    constexpr double MIN_HORIZONTAL_SPREAD = 2.0;  // [m]

    const auto gnssFrames = extract_gnss_frames_from_sm(sm, geoRef);
    if (gnssFrames.frames.empty())
    {
        return std::nullopt;
    }
    if (!geoRef.has_value())
    {
        geoRef = gnssFrames.refCoord;
    }

    std::vector<mrpt::math::TPoint3D> antennaInMap;
    std::vector<mrpt::math::TPoint3D> antennaInEnu;
    for (const auto& gf : gnssFrames.frames)
    {
        const auto frameId = static_cast<size_t>(gf.kf_index);
        if (frameId >= sm.size())
        {
            continue;
        }
        if (!gnss_reading_accepted(gf, params))
        {
            continue;
        }
        const auto&                antennaPose = gf.obs->sensorPose;
        const mrpt::math::TPoint3D antennaOnVehicle(
            antennaPose.x(), antennaPose.y(), antennaPose.z());
        antennaInMap.push_back(kfPose(frameId).composePoint(antennaOnVehicle));
        antennaInEnu.push_back(gf.enu);
    }

    auto fit = fit_yaw_translation(antennaInMap, antennaInEnu, MIN_HORIZONTAL_SPREAD);
    if (logger)
    {
        if (fit)
        {
            logger->logFmt(
                mrpt::system::LVL_INFO,
                "[gnss_factor_helpers] Initial yaw+translation alignment from %zu GNSS readings: "
                "%s",
                antennaInMap.size(), fit->asString().c_str());
        }
        else
        {
            logger->logFmt(
                mrpt::system::LVL_WARN,
                "[gnss_factor_helpers] No initial GNSS alignment: %zu usable readings, too few or "
                "too close together.",
                antennaInMap.size());
        }
    }
    return fit;
}
