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

#include <mrpt/poses/CPose3D.h>

#include <cstddef>
#include <vector>

namespace mola::lc_common
{
/** A pose with its timestamp [s] */
struct StampedPose
{
    double               stamp = 0;
    mrpt::poses::CPose3D pose;
};

struct DensifyResult
{
    /** [s] Constant offset added to the keyframe stamps to match the dense ones */
    double time_offset = 0;
    /** Keyframes found in the dense trajectory (within 20 ms), for the check below */
    std::size_t matched_keyframes = 0;
    /** [m] Largest distance between a corrected keyframe and its densified pose */
    double max_error_at_keyframes = 0;
};

/** Applies the corrections of a keyframe-only re-optimization to a dense
 *  trajectory of the same run (e.g. the odometry output, in the frame of
 *  \a kfBefore).
 *
 *  For each keyframe k, the correction C_k = after_k * inv(before_k) is
 *  interpolated in time (translation linearly, rotation by slerp) and applied to
 *  every dense pose: out(t) = C(t) * dense(t). Before the first or after the last
 *  keyframe, the nearest correction is used.
 *
 *  Keyframes and dense poses may be stamped at different points of a scan (e.g.
 *  its start and its middle): the median time from each keyframe to its nearest
 *  dense pose is estimated first and added to the keyframe stamps, if it is
 *  below \a maxTimeOffset.
 *
 *  \a kfBefore and \a kfAfter must have the same stamps, sorted; so must \a dense.
 */
DensifyResult densify_trajectory(
    const std::vector<StampedPose>& kfBefore, const std::vector<StampedPose>& kfAfter,
    const std::vector<StampedPose>& dense, std::vector<StampedPose>& out,
    double maxTimeOffset = 0.5);

}  // namespace mola::lc_common
