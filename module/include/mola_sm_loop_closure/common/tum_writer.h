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

#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/COutputLogger.h>

#include <functional>
#include <string>

namespace mola::lc_common
{
/** Save the trajectory from a CSimpleMap to a TUM-format text file.
 *
 * @param filename   Output path for the .tum file.
 * @param sm         Source simplemap (used for timestamps and frame count).
 * @param poseOf     Callable returning the optimized SE(3) pose for keyframe index i.
 * @param covOf      Optional callable returning the 6x6 pose covariance for KF i.
 *                   When provided, per-frame sigmas are written to a .cov sidecar.
 * @param logger     Optional logger for info/warn messages.
 *
 * Keyframes whose sensory frame is empty are skipped (bug §7-4 fix).
 */
void save_trajectory_as_tum(
    const std::string& filename, const mrpt::maps::CSimpleMap& sm,
    const std::function<mrpt::poses::CPose3D(size_t)>&        poseOf,
    const std::function<mrpt::math::CMatrixDouble66(size_t)>* covOf  = nullptr,
    mrpt::system::COutputLogger*                              logger = nullptr);

}  // namespace mola::lc_common
