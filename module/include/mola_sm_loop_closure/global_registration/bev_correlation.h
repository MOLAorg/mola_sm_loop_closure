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

#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3D.h>

namespace mola::global_registration
{
/** Parameters of bev_correlation(). */
struct BevCorrelationParameters
{
    /// Cell size of the bird's eye view grids [m].
    double grid_resolution = 1.0;

    /** Points farther than this (horizontally) from each cloud origin are
     *  ignored [m]. The square grids are sized to contain that circle. */
    double max_range = 64.0;

    /// Yaw search step [deg], of the fine stage if there are two.
    double yaw_step_deg = 1.5;

    /** Coarse-to-fine search: a first pass over every yaw on a grid this many
     *  times coarser, every coarse_yaw_step_deg; then, at the full resolution,
     *  yaw_step_deg steps around the best coarse_candidates (at least 2, for
     *  the ambiguity ratio). coarse_resolution_factor 1: one stage. */
    unsigned int coarse_resolution_factor = 2;
    double       coarse_yaw_step_deg      = 6.0;
    size_t       coarse_candidates        = 3;

    /// Cell vertical extents are clamped to this [m].
    double max_cell_height = 3.0;

    /// Minimum normalized correlation of the best peak to accept a result.
    double min_score = 0.25;

    /// Minimum ratio of the best peak to the best clearly different one.
    double min_ambiguity_ratio = 1.25;

    /// Peaks closer than these to the best one are not alternatives [m], [deg].
    double ambiguity_min_distance = 3.0;
    double ambiguity_min_yaw_deg  = 10.0;

    /// Upper limit of 2 * max_range / grid_resolution (memory and time).
    static constexpr size_t MAX_GRID_CELLS = 1024;

    /// Loads the given values (others keep their defaults), then validate().
    void load_from_yaml(const mrpt::containers::yaml& c);

    /// Throws if a value is out of range.
    void validate() const;
};

/** Output of bev_correlation(). */
struct BevCorrelationResult
{
    /// True if the score and ambiguity ratio pass their thresholds.
    bool valid = false;

    /// Pose of the local cloud frame in the global cloud frame.
    mrpt::poses::CPose3D pose;

    /// Normalized correlation of the best peak, in [-1, 1].
    double score = 0;

    /// Best score divided by the best one at a clearly different pose.
    double ambiguity_ratio = 0;
};

/** Global registration of two point clouds, without an initial guess, for
 *  loop closure: a 4-DoF (x, y, yaw, then z) search that assumes both clouds'
 *  roll and pitch are known (e.g. from gravity-aligned odometry).
 *
 *  Both clouds are leveled with their attitudes (only roll and pitch are used),
 *  projected into bird's eye view grids of the cell vertical extent, and
 *  matched by cross-correlation (computed with FFT) for each yaw step. z comes
 *  from the median ground height difference in the overlap. The result is
 *  coarse (about one cell and one yaw step): refine it with ICP.
 *
 * \param global, local Point clouds, each in its own sensor or vehicle frame.
 * \param globalAttitude, localAttitude Poses of those frames in a gravity
 *        aligned frame (only roll and pitch are used).
 */
BevCorrelationResult bev_correlation(
    const mrpt::maps::CPointsMap& global, const mrpt::poses::CPose3D& globalAttitude,
    const mrpt::maps::CPointsMap& local, const mrpt::poses::CPose3D& localAttitude,
    const BevCorrelationParameters& params);

}  // namespace mola::global_registration
