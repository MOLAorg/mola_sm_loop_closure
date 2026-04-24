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

#include <mrpt/obs/CSensoryFrame.h>

#include <optional>

namespace mola::lc_common
{
bool frame_has_mapping_observations(const mrpt::obs::CSensoryFrame& sf);

std::optional<mrpt::Clock::time_point> sf_timestamp(const mrpt::obs::CSensoryFrame& sf);

}  // namespace mola::lc_common
