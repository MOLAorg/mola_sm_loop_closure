// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
// Licensed under the GNU GPL v3.
//
// This file is part of MOLA.
// MOLA is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// MOLA. If not, see <https://www.gnu.org/licenses/>.
//
// Closed-source licenses available upon request, for this odometry package
// alone or in combination with the complete SLAM system.
// -----------------------------------------------------------------------------

/**
 * @file   DeterministicScope.h
 * @brief  Pins every parallel runtime this library reaches to one thread.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/system/COutputLogger.h>

#include <string>

#if defined(MOLA_SM_LC_HAS_TBB)
#include <tbb/global_control.h>
#endif

#if defined(MOLA_SM_LC_HAS_OPENMP)
#include <omp.h>
#endif

#include <optional>

namespace mola
{
/** RAII: while alive, every parallel runtime the loop-closure pipeline can
 *  reach runs on one thread, and the previous settings are restored on exit.
 *
 *  WHY A SCOPE AND NOT A FLAG. Making loop closure reproducible needs more than
 *  evaluating candidates sequentially, because the nondeterminism is not only
 *  in WHICH thread takes WHICH candidate. Underneath a single candidate's ICP:
 *
 *  - mp2p_icp builds its pairing list with `tbb::parallel_reduce` and joins the
 *    per-range results by concatenation, so the ORDER of the resulting
 *    correspondences follows the reduction tree, which TBB shapes according to
 *    how many workers happened to be free.
 *  - `optimal_tf_gauss_newton` sums H and g over those pairings with another
 *    `parallel_reduce`, so even for a fixed pairing list the floating-point
 *    summation order varies, and the solution differs in its last bits.
 *  - KISS-Matcher, when enabled as the global-registration front end, uses BOTH
 *    TBB and OpenMP (`#pragma omp parallel for` in ROBINMatching).
 *
 *  None of those read this library's own thread parameters, and the last one
 *  would not be covered by a TBB setting alone. With one thread each, TBB's
 *  recursive range split executes depth-first with no stealing, so the join
 *  order is fixed and the reduction becomes a function of the input.
 *
 *  COST. This is a process-wide setting for as long as the scope lives, so it
 *  also throttles anything else running concurrently. That is acceptable where
 *  it is used -- a batch/offline run whose point is reproducibility -- and is
 *  the reason it is scoped to the analysis rather than installed at startup.
 */
class DeterministicScope
{
   public:
    /** @param enabled  when false the object does nothing at all, so callers
     *                  can construct it unconditionally.
     *  @param logger   optional, to report what could NOT be pinned. */
    explicit DeterministicScope(bool enabled, mrpt::system::COutputLogger* logger = nullptr)
        : enabled_(enabled)
    {
        if (!enabled_)
        {
            return;
        }

#if defined(MOLA_SM_LC_HAS_TBB)
        tbbControl_.emplace(tbb::global_control::max_allowed_parallelism, 1);
#else
        unpinned_ += "TBB ";
#endif

#if defined(MOLA_SM_LC_HAS_OPENMP)
        ompPrevThreads_ = omp_get_max_threads();
        omp_set_num_threads(1);
#else
        unpinned_ += "OpenMP ";
#endif

        // Not silent about a half-done job: a caller that asked for
        // reproducibility and cannot get it must be told, not left to discover
        // it by diffing two outputs.
        if (!unpinned_.empty() && logger != nullptr)
        {
            logger->logStr(
                mrpt::system::LVL_WARN,
                "[deterministic] this build cannot pin: " + unpinned_ +
                    "-- results may still vary run to run. Rebuild with those "
                    "found by CMake for a fully reproducible scan.");
        }
    }

    ~DeterministicScope()
    {
        if (!enabled_)
        {
            return;
        }
#if defined(MOLA_SM_LC_HAS_OPENMP)
        if (ompPrevThreads_ > 0)
        {
            omp_set_num_threads(ompPrevThreads_);
        }
#endif
        // tbbControl_ restores the previous limit on destruction.
    }

    DeterministicScope(const DeterministicScope&)            = delete;
    DeterministicScope& operator=(const DeterministicScope&) = delete;

    /** True when every runtime this build knows about was pinned. */
    [[nodiscard]] bool fully_pinned() const { return !enabled_ || unpinned_.empty(); }

   private:
    bool        enabled_ = false;
    std::string unpinned_;

#if defined(MOLA_SM_LC_HAS_TBB)
    std::optional<tbb::global_control> tbbControl_;
#endif
#if defined(MOLA_SM_LC_HAS_OPENMP)
    int ompPrevThreads_ = 0;
#endif
};

}  // namespace mola
