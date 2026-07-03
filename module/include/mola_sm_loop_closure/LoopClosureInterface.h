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

#include <cstdint>
#include <vector>

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
     *  Not every engine supports this; the base implementation throws. The
     *  snapshot must outlive the call; the engine must not retain references to
     *  it afterwards.
     */
    virtual std::vector<ProposedLoopEdge> analyze(const mrpt::maps::CSimpleMap& snapshot);

    /** @} */
};

}  // namespace mola