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
#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <optional>
#include <utility>

namespace mola::lc_common
{
/** Rebuild planarity factor graph: (z,roll,pitch) soft priors on every X(i).
 *  GTSAM Pose3 tangent-space order: [rx, ry, rz, tx, ty, tz].
 *  Planarity constrains tz and rx,ry (roll,pitch); leaves x,y,yaw free. */
void build_planarity_factors(
    gtsam::NonlinearFactorGraph& out, const gtsam::Values& currentValues, std::size_t nPoses,
    double sigmaZ, double sigmaAng);

struct PlanarAnneal
{
    double initSigmaZ   = 0.10;  // [m] sigma at round 0
    double initSigmaAng = 0.02;  // [rad] sigma at round 0
    size_t rounds       = 2;  // rounds until constraint vanishes
};

/** Returns {sigmaZ, sigmaAng} for the given LC round, or nullopt when the
 *  constraint should be dropped (lcRound >= rounds). */
std::optional<std::pair<double, double>> planar_sigmas_for_round(
    const PlanarAnneal& p, std::size_t lcRound);

}  // namespace mola::lc_common
