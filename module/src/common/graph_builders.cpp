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
#include <gtsam/slam/BetweenFactor.h>
#include <mola_sm_loop_closure/common/graph_builders.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/gtsam_wrappers.h>

void mola::lc_common::add_odometry_edges(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& values, const mrpt::maps::CSimpleMap& sm,
    const OdometryEdgeParams& params, std::vector<uint64_t>& knownInliers)
{
    using gtsam::symbol_shorthand::X;

    // Insert initial pose estimates for all keyframes not yet in values.
    for (size_t i = 0; i < sm.size(); i++)
    {
        if (!values.exists(X(i)))
        {
            const auto& [pose_i, sf_i, twist_i] = sm.get(i);
            mrpt::poses::CPose3DPDFGaussian p;
            p.copyFrom(*pose_i);
            values.insert(X(i), mrpt::gtsam_wrappers::toPose3(p.mean));
        }
    }

    // One BetweenFactor<Pose3> per consecutive pair.
    for (size_t i = 1; i < sm.size(); i++)
    {
        const auto& [pose_i, sf_i, twist_i]       = sm.get(i);
        const auto& [pose_im1, sf_im1, twist_im1] = sm.get(i - 1);
        mrpt::poses::CPose3DPDFGaussian ppi;
        mrpt::poses::CPose3DPDFGaussian ppim1;
        ppi.copyFrom(*pose_i);
        ppim1.copyFrom(*pose_im1);

        const mrpt::poses::CPose3DPDFGaussian relPose = ppi - ppim1;

        gtsam::Vector6 sigmasXYZYPR =
            relPose.cov.asEigen().diagonal().cwiseMax(0.0).array().sqrt().eval();

        sigmasXYZYPR *= params.uncertainty_multiplier;

        for (int k = 0; k < 3; k++)
        {
            sigmasXYZYPR[k] += params.additional_noise_xyz;
            sigmasXYZYPR[3 + k] += mrpt::DEG2RAD(params.additional_noise_ang_deg);
        }

        // GTSAM Pose3 tangent-space order: (Rx, Ry, Rz, tx, ty, tz)
        gtsam::Vector6 sigmas;
        sigmas << sigmasXYZYPR[5], sigmasXYZYPR[4], sigmasXYZYPR[3], sigmasXYZYPR[0],
            sigmasXYZYPR[1], sigmasXYZYPR[2];

        auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

        const gtsam::Pose3 deltaPose = mrpt::gtsam_wrappers::toPose3(relPose.getPoseMean());

        knownInliers.push_back(static_cast<uint64_t>(fg.size()));
        fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), deltaPose, edgeNoise);
    }
}
