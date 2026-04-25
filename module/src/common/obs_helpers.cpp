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
#include <mola_sm_loop_closure/common/obs_helpers.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>

namespace mola::lc_common
{
bool frame_has_mapping_observations(const mrpt::obs::CSensoryFrame& sf)
{
    if (sf.getObservationByClass<mrpt::obs::CObservationPointCloud>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservation2DRangeScan>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservationVelodyneScan>())
    {
        return true;
    }
    return false;
}

std::optional<mrpt::Clock::time_point> sf_timestamp(const mrpt::obs::CSensoryFrame& sf)
{
    for (const auto& o : sf)
    {
        if (!o)
        {
            continue;
        }
        if (o->timestamp == mrpt::Clock::time_point())
        {
            continue;
        }
        return o->timestamp;
    }
    return {};
}

}  // namespace mola::lc_common
