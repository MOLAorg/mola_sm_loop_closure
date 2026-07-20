/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <utility>
#include <vector>

/** Feature macro: when defined, LoopClosureAnalyzeOptions exposes on_progress /
 *  out_stats (and the LoopClosureAnalyzeStats summary), so a consumer can report
 *  per-scan progress and candidate/accept counts. Lets out-of-repo consumers
 *  detect the feature at compile time. */
#define MOLA_SM_LOOP_CLOSURE_HAS_ANALYZE_STATS 1

namespace mola
{
/** A loop-closure edge proposed by a detector, decoupled from any factor graph.
 *
 * Producers return these from LoopClosureInterface::analyze() without owning or
 * mutating the input map; the consumer (e.g. the central mapper) decides
 * whether and how to merge them into its own optimizer.
 */
struct ProposedLoopEdge
{
    /// Source keyframe index in the analyzed simplemap.
    uint32_t from = 0;

    /// Target keyframe index in the analyzed simplemap.
    uint32_t to = 0;

    /// Relative pose of `to` as seen from `from` (i.e. pose_to = pose_from (+)
    /// relative_pose), with its covariance.
    mrpt::poses::CPose3DPDFGaussian relative_pose;

    /// Detector confidence in [0,1] (e.g. ICP goodness).
    double quality = 0;
};

/** Summary counts of one LoopClosureInterface::analyze() pass, so the caller
 *  can report progress/statistics (e.g. in a UI) without re-deriving them.
 *  Rejected candidates = candidates_evaluated - edges_accepted. */
struct LoopClosureAnalyzeStats
{
    /// Candidate pairs the detector decided to test this pass (after candidate
    /// selection / exclusion), i.e. the initial "queue" size for the scan.
    std::size_t candidates_generated = 0;

    /// Candidates whose ICP actually ran (== candidates_generated unless the
    /// scan was aborted early).
    std::size_t candidates_evaluated = 0;

    /// Candidates accepted as loop-closure edges (passed min_icp_goodness).
    std::size_t edges_accepted = 0;

    /// True if the pass stopped early via should_abort().
    bool aborted = false;
};

/** Options for the streaming / incremental LoopClosureInterface::analyze(). */
struct LoopClosureAnalyzeOptions
{
    /** If set, only consider candidate pairs whose LATER keyframe index is
     *  >= this value, i.e. pairs that involve at least one keyframe added since
     *  the previous call. Pairs fully below this index are assumed already
     *  evaluated, which skips the bulk of the ICP work as the map grows.
     *  Leave unset for a full scan of all pairs.
     *
     *  Caveat: if the consumer's re-optimization moves old keyframes enough
     *  that a previously-implausible old/old pair becomes a loop, the hint
     *  hides it; do an occasional full scan (unset) to recover such loops. */
    std::optional<uint32_t> first_new_keyframe;

    /** If set, invoked for each accepted edge the moment it is found, so the
     *  consumer can merge loops early instead of waiting for analyze() to
     *  return. Called from the analyze() thread; must be thread-safe and must
     *  not call back into this engine. The edge is also included in the
     *  returned vector. */
    std::function<void(const ProposedLoopEdge&)> on_edge_found;

    /** If set, polled in the candidate loop before each (expensive) ICP; return
     *  true to stop early and return the edges found so far. Lets the consumer
     *  cancel a long-running scan on new data or shutdown. */
    std::function<bool()> should_abort;

    /** Keyframe-index pairs (min,max) to exclude from candidate selection, e.g.
     *  loops the consumer already closed. Excluding them frees the per-scan
     *  candidate budget for as-yet-unclosed pairs, so repeated full scans keep
     *  discovering new loops instead of re-proposing the same ones. */
    std::set<std::pair<uint32_t, uint32_t>> exclude_pairs;

    /** If set, invoked as candidates are evaluated with (done, total), so the
     *  consumer can show live progress / a per-scan pending-queue depth
     *  (total - done) while the (possibly long) ICP scan runs. Called from the
     *  analyze() thread(s); must be thread-safe and must not call back into
     *  this engine. `total` is the candidate count for the pass (constant);
     *  `done` is monotonically increasing. */
    std::function<void(std::size_t done, std::size_t total)> on_progress;

    /** If set, filled with the pass summary counts before analyze() returns. */
    LoopClosureAnalyzeStats* out_stats = nullptr;
};

class LoopClosureInterface : public mrpt::rtti::CObject, public mrpt::system::COutputLogger
{
    DEFINE_VIRTUAL_MRPT_OBJECT(LoopClosureInterface, mola)

   public:
    LoopClosureInterface();
    virtual ~LoopClosureInterface();

    // Disable copy and move operations
    LoopClosureInterface(const LoopClosureInterface&)            = delete;
    LoopClosureInterface& operator=(const LoopClosureInterface&) = delete;
    LoopClosureInterface(LoopClosureInterface&&)                 = delete;
    LoopClosureInterface& operator=(LoopClosureInterface&&)      = delete;

    /** @name Main API
     * @{ */

    virtual void initialize(const mrpt::containers::yaml& cfg) = 0;

    /** Find and apply loop closures in the input/output simplemap */
    virtual void process(mrpt::maps::CSimpleMap& sm) = 0;

    /** Detector-only counterpart of process(): find loop-closure candidates in
     *  a read-only map snapshot and return the proposed edges, WITHOUT building
     *  or optimizing an internal factor graph and WITHOUT mutating the map. The
     *  caller owns the snapshot and merges the returned edges into its own
     *  optimizer.
     *
     *  Optionally streams accepted edges early, restricts the search to
     *  newly-added keyframes, and can be aborted mid-scan (see
     *  LoopClosureAnalyzeOptions).
     *
     *  Not every engine supports this; the base implementation throws. The
     *  snapshot must outlive the call; the engine must not retain references to
     *  it afterwards.
     *
     *  An instance used for analyze() should not also be used for process():
     *  the two entry points drive shared internal state independently and are
     *  not meant to be interleaved on the same object.
     */
    virtual std::vector<ProposedLoopEdge> analyze(
        const mrpt::maps::CSimpleMap& snapshot, const LoopClosureAnalyzeOptions& opts = {});

    /** @} */
};

}  // namespace mola