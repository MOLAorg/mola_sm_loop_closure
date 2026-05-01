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
#include <mola_sm_loop_closure/common/debug_flags.h>
#include <mrpt/core/get_env.h>

namespace mola::lc_common
{
DebugFlags::DebugFlags()
    : print_lc_scores(mrpt::get_env<bool>("PRINT_LC_SCORES", false)),
      save_icp_logs(mrpt::get_env<bool>("SAVE_ICP_LOGS", false)),
      print_all_scores(mrpt::get_env<bool>("PRINT_ALL_SCORES", false)),
      save_lcs(mrpt::get_env<bool>("SAVE_LCS", false)),
      save_trees(mrpt::get_env<bool>("SAVE_TREES", false)),
      print_fg_errors(mrpt::get_env<bool>("PRINT_FG_ERRORS", false)),
      debug_print_between_edges(mrpt::get_env<bool>("DEBUG_PRINT_BETWEEN_EDGES", false))
{
}

const DebugFlags& DebugFlags::instance()
{
    static const DebugFlags inst;
    return inst;
}

}  // namespace mola::lc_common
