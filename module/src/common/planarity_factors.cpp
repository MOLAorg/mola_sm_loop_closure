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
#include <gtsam/slam/PriorFactor.h>
#include <mola_sm_loop_closure/common/planarity_factors.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <cmath>

namespace mola::lc_common
{
void build_planarity_factors(
    gtsam::NonlinearFactorGraph& out, const gtsam::Values& currentValues, std::size_t nPoses,
    double sigmaZ, double sigmaAng)
{
    using gtsam::symbol_shorthand::X;

    out.resize(0);

    constexpr double kLargeSigma = 1e6;

    for (std::size_t i = 0; i < nPoses; i++)
    {
        const auto pose = mrpt::poses::CPose3D(
            mrpt::gtsam_wrappers::toTPose3D(currentValues.at<gtsam::Pose3>(X(i))));

        const mrpt::poses::CPose3D target(pose.x(), pose.y(), 0.0, pose.yaw(), 0.0, 0.0);

        gtsam::Vector6 sigmas;
        sigmas << sigmaAng, sigmaAng, kLargeSigma,  // rx(roll), ry(pitch), rz(yaw): free yaw
            kLargeSigma, kLargeSigma, sigmaZ;  // tx, ty: free; tz: constrained

        auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        out.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            X(i), mrpt::gtsam_wrappers::toPose3(target), noise);
    }
}

std::optional<std::pair<double, double>> planar_sigmas_for_round(
    const PlanarAnneal& p, std::size_t lcRound)
{
    ASSERT_GT_(p.initSigmaZ, 0.0);
    ASSERT_GT_(p.initSigmaAng, 0.0);
    if (lcRound >= p.rounds)
    {
        return std::nullopt;
    }
    const double t        = static_cast<double>(lcRound) / static_cast<double>(p.rounds);
    const double logScale = std::log(1e6);
    const double sigmaZ   = p.initSigmaZ * std::exp(t * (logScale - std::log(p.initSigmaZ)));
    const double sigmaAng = p.initSigmaAng * std::exp(t * (logScale - std::log(p.initSigmaAng)));
    return std::make_pair(sigmaZ, sigmaAng);
}

}  // namespace mola::lc_common
