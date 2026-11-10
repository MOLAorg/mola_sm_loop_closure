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

#include <mola_sm_loop_closure/common/densify_trajectory.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/slerp.h>

#include <algorithm>
#include <cmath>

namespace
{
/// Index of the stamp in \a stamps closest to \a t (stamps sorted, non-empty)
std::size_t closest_index(const std::vector<double>& stamps, double t)
{
    const auto it = std::lower_bound(stamps.begin(), stamps.end(), t);
    if (it == stamps.begin())
    {
        return 0;
    }
    if (it == stamps.end())
    {
        return stamps.size() - 1;
    }
    const auto i = static_cast<std::size_t>(it - stamps.begin());
    return (t - stamps[i - 1] < stamps[i] - t) ? i - 1 : i;
}
}  // namespace

mola::lc_common::DensifyResult mola::lc_common::densify_trajectory(
    const std::vector<StampedPose>& kfBefore, const std::vector<StampedPose>& kfAfter,
    const std::vector<StampedPose>& dense, std::vector<StampedPose>& out, double maxTimeOffset)
{
    ASSERT_EQUAL_(kfBefore.size(), kfAfter.size());
    ASSERT_(!kfBefore.empty());
    ASSERT_(!dense.empty());

    constexpr double STAMP_TOLERANCE = 1e-3;  // [s]
    constexpr double MATCH_TOLERANCE = 0.02;  // [s]

    const std::size_t nKf = kfBefore.size();

    std::vector<double> denseStamps;
    denseStamps.reserve(dense.size());
    for (const auto& p : dense)
    {
        denseStamps.push_back(p.stamp);
    }

    // Keyframe-to-dense time offset:
    DensifyResult       ret;
    std::vector<double> offsets;
    offsets.reserve(nKf);
    for (std::size_t k = 0; k < nKf; k++)
    {
        ASSERTMSG_(
            std::abs(kfBefore[k].stamp - kfAfter[k].stamp) < STAMP_TOLERANCE,
            "Keyframe stamps before and after the optimization do not match");
        offsets.push_back(
            denseStamps[closest_index(denseStamps, kfBefore[k].stamp)] - kfBefore[k].stamp);
    }
    std::nth_element(
        offsets.begin(), offsets.begin() + static_cast<std::ptrdiff_t>(nKf / 2), offsets.end());
    const double medianOffset = offsets[nKf / 2];
    if (std::abs(medianOffset) < maxTimeOffset)
    {
        ret.time_offset = medianOffset;
    }

    // Per-keyframe corrections, at the (shifted) keyframe stamps:
    std::vector<double>              kfStamps;
    std::vector<mrpt::math::TPose3D> corrections;
    kfStamps.reserve(nKf);
    corrections.reserve(nKf);
    for (std::size_t k = 0; k < nKf; k++)
    {
        kfStamps.push_back(kfBefore[k].stamp + ret.time_offset);
        corrections.push_back((kfAfter[k].pose + (-kfBefore[k].pose)).asTPose());
    }

    out.clear();
    out.reserve(dense.size());
    for (const auto& d : dense)
    {
        mrpt::math::TPose3D c;
        const auto          it = std::upper_bound(kfStamps.begin(), kfStamps.end(), d.stamp);
        if (it == kfStamps.begin())
        {
            c = corrections.front();
        }
        else if (it == kfStamps.end())
        {
            c = corrections.back();
        }
        else
        {
            const auto   b     = static_cast<std::size_t>(it - kfStamps.begin());
            const auto   a     = b - 1;
            const double span  = kfStamps[b] - kfStamps[a];
            const double alpha = span > 0 ? (d.stamp - kfStamps[a]) / span : 0.0;
            mrpt::math::slerp(corrections[a], corrections[b], alpha, c);
        }
        out.push_back({d.stamp, mrpt::poses::CPose3D(c) + d.pose});
    }

    // Check: densified poses at keyframe times reproduce the keyframes.
    for (std::size_t k = 0; k < nKf; k++)
    {
        const auto i = closest_index(denseStamps, kfStamps[k]);
        if (std::abs(denseStamps[i] - kfStamps[k]) > MATCH_TOLERANCE)
        {
            continue;
        }
        ret.matched_keyframes++;
        const double err           = out[i].pose.distanceTo(kfAfter[k].pose);
        ret.max_error_at_keyframes = std::max(ret.max_error_at_keyframes, err);
    }
    return ret;
}
