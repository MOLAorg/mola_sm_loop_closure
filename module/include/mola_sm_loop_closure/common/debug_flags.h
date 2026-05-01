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

namespace mola::lc_common
{
/**
 * Debug env-var flags shared between FrameToFrameLoopClosure and
 * SimplemapLoopClosure. Each flag is loaded once at program startup from the
 * corresponding environment variable. Set the variable to "1" or "true" to
 * enable.
 *
 * Available flags:
 *   PRINT_LC_SCORES          - F2F: print per-candidate LC quality scores.
 *   SAVE_ICP_LOGS            - F2F: save ICP log files for every attempt.
 *   PRINT_ALL_SCORES         - SM: print all candidate ranking scores.
 *   SAVE_LCS                 - SM: dump accepted LC edges to disk.
 *   SAVE_TREES               - SM: dump Dijkstra spanning trees to disk.
 *   PRINT_FG_ERRORS          - SM: print large factor-graph errors after GNC.
 *   DEBUG_PRINT_BETWEEN_EDGES - SM: log every BetweenFactor added.
 */
struct DebugFlags
{
    // F2F flags
    bool print_lc_scores = false;
    bool save_icp_logs   = false;

    // SM flags
    bool print_all_scores          = false;
    bool save_lcs                  = false;
    bool save_trees                = false;
    bool print_fg_errors           = false;
    bool debug_print_between_edges = false;

    static const DebugFlags& instance();

   private:
    DebugFlags();
};

}  // namespace mola::lc_common
