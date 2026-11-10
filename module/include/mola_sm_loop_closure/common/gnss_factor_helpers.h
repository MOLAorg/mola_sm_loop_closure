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

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/topography/data_types.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace mola::lc_common
{
struct GnssFactorParams
{
    bool   add_horizontality       = false;
    double horizontality_sigma_rpy = 0.01;  // [rad] roll/pitch-constraint sigma
    double minimum_uncertainty_xyz = 0.10;  // [m] per-axis floor on GPS sigma
    double uncertainty_multiplier  = 1.0;  // applied to sigma before the floor
    double max_uncertainty_horiz   = 20.0;  // [m] reject reading if sqrt(sigE²+sigN²) > this
    double max_uncertainty_vert    = 40.0;  // [m] reject reading if sigU > this
};

/** Add per-keyframe FactorGnssEnu factors to \a fg from GNSS observations in
 *  the simplemap.  Uses \a kf_index directly — no SM re-scan (bug §7-7 fix).
 *
 * @param fg             Target factor graph (X(kf_index) keys).
 * @param sm             Source simplemap.
 * @param geoRef         ENU reference coordinates; set on first call if empty.
 * @param params         Noise model parameters.
 * @param knownInliers   Appended with the new factor indices.
 * @param logger         Optional logger.
 * @return               Number of factors added.
 */
std::size_t add_gnss_factors_per_kf(
    gtsam::NonlinearFactorGraph& fg, const mrpt::maps::CSimpleMap& sm,
    std::optional<mrpt::topography::TGeodeticCoords>& geoRef, const GnssFactorParams& params,
    std::vector<uint64_t>& knownInliers, mrpt::system::COutputLogger* logger = nullptr);

/** Least-squares yaw + translation (4-DoF) that maps the points `src` onto `dst`:
 *  dst_i ~ R_z(yaw) * src_i + t. Roll and pitch are not estimated: `src` is
 *  expected to come from a gravity-aligned odometry.
 *  Returns std::nullopt if there are fewer than 3 pairs, or if the horizontal RMS
 *  spread of `src` around its centroid is below `minHorizontalSpread` [m], since
 *  the yaw would then be unobservable.
 */
std::optional<mrpt::poses::CPose3D> fit_yaw_translation(
    const std::vector<mrpt::math::TPoint3D>& src, const std::vector<mrpt::math::TPoint3D>& dst,
    double minHorizontalSpread);

/** Initial guess for a GNSS-constrained graph: the 4-DoF transform (see
 *  fit_yaw_translation()) that maps the GNSS antenna positions, as given by the
 *  keyframe poses `kfPose(i)` and the receiver pose on the vehicle, onto their ENU
 *  readings. The readings are accepted with the same rules as in
 *  add_gnss_factors_per_kf(), and `geoRef` is set in the same way, so the result is
 *  expressed in the ENU frame those factors will use.
 */
std::optional<mrpt::poses::CPose3D> fit_yaw_translation_to_gnss(
    const mrpt::maps::CSimpleMap& sm, std::optional<mrpt::topography::TGeodeticCoords>& geoRef,
    const GnssFactorParams& params, const std::function<mrpt::poses::CPose3D(size_t)>& kfPose,
    mrpt::system::COutputLogger* logger = nullptr);

}  // namespace mola::lc_common
