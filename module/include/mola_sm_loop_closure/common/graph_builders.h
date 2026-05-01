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
#include <mrpt/maps/CSimpleMap.h>

#include <cstdint>
#include <vector>

namespace mola::lc_common
{
struct OdometryEdgeParams
{
    double additional_noise_xyz     = 1e-3;  // [m] added to simplemap covariance sigma
    double additional_noise_ang_deg = 1e-3;  // [deg] added to simplemap covariance sigma
    double uncertainty_multiplier   = 1.0;  // applied to sigma before adding floor
};

/** Build pose-graph odometry edges from a CSimpleMap.
 *
 * For each keyframe i, inserts X(i) into \a values from the simplemap pose if
 * not already present.  For each consecutive pair (i-1, i), emits a
 * BetweenFactor<Pose3> whose noise model is derived from the simplemap relative
 * covariance scaled by \a params.uncertainty_multiplier plus the per-axis
 * additional-noise floors.  Factor indices are appended to \a knownInliers.
 *
 * Note: the prior on X(0) is NOT added here — the caller is responsible for it.
 */
void add_odometry_edges(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& values, const mrpt::maps::CSimpleMap& sm,
    const OdometryEdgeParams& params, std::vector<uint64_t>& knownInliers);

}  // namespace mola::lc_common
