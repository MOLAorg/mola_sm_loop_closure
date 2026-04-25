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
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <mola_sm_loop_closure/common/gnc_optimizer.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <algorithm>

namespace mola::lc_common
{
GncResult run_gnc(
    const gtsam::NonlinearFactorGraph& mainFG, const gtsam::NonlinearFactorGraph& planarityFG,
    const gtsam::Values& initialValues, const std::vector<uint64_t>& knownInlierIndices,
    mrpt::system::COutputLogger* /*logger*/)
{
    gtsam::NonlinearFactorGraph combined = mainFG;
    const std::size_t           nBase    = mainFG.size();
    if (!planarityFG.empty())
    {
        combined.add(planarityFG);
    }

    if (combined.empty())
    {
        return {};
    }

    const double N_1      = 1.0 / static_cast<double>(combined.size());
    const double errInit  = combined.error(initialValues);
    const double rmseInit = std::sqrt(errInit * N_1);

    using GncParams    = gtsam::GncParams<gtsam::LevenbergMarquardtParams>;
    auto      lmParams = gtsam::LevenbergMarquardtParams::CeresDefaults();
    GncParams gncParams(lmParams);
    gncParams.setLossType(gtsam::GncLossType::GM);

    GncParams::IndexVector knownInliers(knownInlierIndices.begin(), knownInlierIndices.end());
    for (std::size_t k = 0; k < planarityFG.size(); k++)
    {
        knownInliers.push_back(nBase + k);
    }
    std::sort(knownInliers.begin(), knownInliers.end());
    knownInliers.erase(std::unique(knownInliers.begin(), knownInliers.end()), knownInliers.end());
    gncParams.setKnownInliers(knownInliers);

    gtsam::GncOptimizer<GncParams> gnc(combined, initialValues, gncParams);
    const auto                     optValues = gnc.optimize();

    const double errEnd  = combined.error(optValues);
    const double rmseEnd = std::sqrt(errEnd * N_1);

    const auto& gncWeights   = gnc.getWeights();
    std::size_t numLcInliers = 0, numLcOutliers = 0;
    for (std::size_t k = 0; k < static_cast<std::size_t>(gncWeights.size()); k++)
    {
        const bool isKnownInlier =
            std::binary_search(knownInliers.begin(), knownInliers.end(), static_cast<uint64_t>(k));
        if (!isKnownInlier)
        {
            if (gncWeights[static_cast<Eigen::Index>(k)] < 0.5)
            {
                numLcOutliers++;
            }
            else
            {
                numLcInliers++;
            }
        }
    }

    // Compute largest pose change:
    double largestDelta = 0.0;
    using gtsam::symbol_shorthand::X;
    for (const auto& key_val : initialValues)
    {
        try
        {
            const auto newPose =
                mrpt::gtsam_wrappers::toTPose3D(optValues.at<gtsam::Pose3>(key_val.key));
            const auto oldPose =
                mrpt::gtsam_wrappers::toTPose3D(initialValues.at<gtsam::Pose3>(key_val.key));
            const double delta = mrpt::poses::CPose3D(oldPose - newPose).translation().norm();
            mrpt::keep_max(largestDelta, delta);
        }
        catch (...)
        {
        }
    }

    GncResult r;
    r.values        = optValues;
    r.rmseInit      = rmseInit;
    r.rmseEnd       = rmseEnd;
    r.largestDelta  = largestDelta;
    r.numLcInliers  = numLcInliers;
    r.numLcOutliers = numLcOutliers;
    return r;
}

}  // namespace mola::lc_common
