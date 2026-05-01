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

#include <mola_sm_loop_closure/common/tum_writer.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>

void mola::lc_common::save_trajectory_as_tum(
    const std::string& filename, const mrpt::maps::CSimpleMap& sm,
    const std::function<mrpt::poses::CPose3D(size_t)>&        poseOf,
    const std::function<mrpt::math::CMatrixDouble66(size_t)>* covOf,
    mrpt::system::COutputLogger*                              logger)
{
    mrpt::poses::CPose3DInterpolator path;
    mrpt::math::CMatrixDouble        pathSigmas;

    for (size_t id = 0; id < sm.size(); id++)
    {
        const auto& [oldPose, sf, twist] = sm.get(id);

        if (!sf || sf->empty())
        {
            if (logger)
            {
                logger->logFmt(
                    mrpt::system::LVL_WARN, "Frame %zu has no observations, skipping in TUM", id);
            }
            continue;
        }

        const auto t = sf->getObservationByIndex(0)->timestamp;
        path.insert(t, poseOf(id));

        if (covOf)
        {
            const auto& cov = (*covOf)(id);
            const auto  row = static_cast<int>(path.size() - 1);
            pathSigmas.conservativeResize(row + 1, 6);
            for (int k = 0; k < 6; k++)
            {
                pathSigmas(row, k) = std::sqrt(cov(k, k));
            }
        }
    }

    path.saveToTextFile_TUM(filename);

    if (covOf && pathSigmas.rows() > 0)
    {
        const std::string header =
            "# sigma_x_m sigma_y_m sigma_z_m sigma_yaw_rad sigma_pitch_rad sigma_roll_rad";
        pathSigmas.saveToTextFile(
            mrpt::system::fileNameChangeExtension(filename, "cov"), mrpt::math::MATRIX_FORMAT_ENG,
            false, header);
    }

    if (logger)
    {
        logger->logFmt(mrpt::system::LVL_INFO, "Saved trajectory to: %s", filename.c_str());
    }
}
