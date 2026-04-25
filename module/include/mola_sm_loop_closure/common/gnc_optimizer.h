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
#include <mrpt/system/COutputLogger.h>

#include <vector>

namespace mola::lc_common
{
struct GncResult
{
    gtsam::Values values;
    double        rmseInit      = 0;
    double        rmseEnd       = 0;
    double        largestDelta  = 0;
    std::size_t   numLcInliers  = 0;
    std::size_t   numLcOutliers = 0;
};

/** Run GNC (Graduated Non-Convexity, GM kernel) optimisation.
 *
 *  \param mainFG        Main factor graph (odometry + GNSS + ICP LC edges).
 *  \param planarityFG   Optional planarity factors (appended as known inliers).
 *  \param initialValues Initial linearisation point.
 *  \param knownInlierIndices Factor indices known to be inliers (odometry, GNSS, planarity).
 */
GncResult run_gnc(
    const gtsam::NonlinearFactorGraph& mainFG, const gtsam::NonlinearFactorGraph& planarityFG,
    const gtsam::Values& initialValues, const std::vector<uint64_t>& knownInlierIndices,
    mrpt::system::COutputLogger* logger = nullptr);

}  // namespace mola::lc_common
