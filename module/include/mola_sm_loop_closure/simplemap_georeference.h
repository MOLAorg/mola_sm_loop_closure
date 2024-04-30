/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/system/COutputLogger.h>

// GTSAM:
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace mola
{
struct SMGeoReferencingOutput
{
    SMGeoReferencingOutput() = default;

    mp2p_icp::metric_map_t::Georeferencing geo_ref;
    double                                 final_rmse = .0;
};

struct AddGNNSFactorParams
{
    /// May be required for small maps, i.e. when the length of the trajectory
    /// is not >10 times the GNNS uncertainty.
    bool   addHorizontalityConstraints = false;
    double horizontalitySigmaZ         = 1.0;  // [m]
};

struct SMGeoReferencingParams
{
    SMGeoReferencingParams() = default;

    /// If provided, this will be the coordinates of the ENU frame origin.
    /// Otherwise (default), the first GNNS entry will become the reference.
    std::optional<mrpt::topography::TGeodeticCoords> geodeticReference;

    AddGNNSFactorParams fgParams;

    mrpt::system::COutputLogger* logger = nullptr;
};

/** Function to georeferencing a given SimpleMap with GNNS observations.
 *  A minimum of 3 (non-colinear) KFs with GNNS data are required to solve
 *  for the optimal transformation.
 */
SMGeoReferencingOutput simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm,
    const SMGeoReferencingParams& params = {});

struct FrameGNNS
{
    mrpt::poses::CPose3D              pose;
    mrpt::obs::CObservationGPS::Ptr   obs;
    mrpt::obs::gnss::Message_NMEA_GGA gga;
    mrpt::topography::TGeodeticCoords coords;
    mrpt::math::TPoint3D              enu;
    double                            sigma_E = 5.0;
    double                            sigma_N = 5.0;
    double                            sigma_U = 5.0;
};

struct GNNSFrames
{
    std::vector<FrameGNNS>                           frames;
    std::optional<mrpt::topography::TGeodeticCoords> refCoord;
};

GNNSFrames extract_gnns_frames_from_sm(
    const mrpt::maps::CSimpleMap&                           sm,
    const std::optional<mrpt::topography::TGeodeticCoords>& refCoord =
        std::nullopt);

void add_gnns_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const GNNSFrames& frames,
    const AddGNNSFactorParams& params);

}  // namespace mola
