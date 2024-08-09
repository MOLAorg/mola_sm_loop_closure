/*
 * Proprietary Software License
 *
 * Copyright (c) 2018-2024 Jose Luis Blanco-Claraco, University of Almeria.
 * All rights reserved.
 *
 * This file is part of MOLA (Modular Optimization framework
 * for Localization and mApping)
 *
 * Unauthorized copying, distribution, modification, or use of this file,
 * via any medium, is strictly prohibited without the prior written permission
 * of University of Almería (PI: Jose Luis Blanco-Claraco).
 *
 * License:
 * You may use this file in accordance with the terms and conditions
 * set forth in the Proprietary Software License, which is included
 * with this software or can be obtained at University of Almeria.
 *
 * Disclaimer:
 * This software is provided "as is," without warranty of any kind,
 * express or implied, including but not limited to the warranties
 * of merchantability, fitness for a particular purpose, and noninfringement.
 * In no event shall the authors or copyright holders be liable
 * for any claim, damages, or other liability, whether in an action
 * of contract, tort, or otherwise, arising from, out of, or in connection
 * with the software or the use or other dealings in the software.
 */

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

struct AddGNSSFactorParams
{
    /// May be required for small maps, i.e. when the length of the trajectory
    /// is not >10 times the GNSS uncertainty.
    bool   addHorizontalityConstraints = false;
    double horizontalitySigmaZ         = 1.0;  // [m]
};

struct SMGeoReferencingParams
{
    SMGeoReferencingParams() = default;

    /// If provided, this will be the coordinates of the ENU frame origin.
    /// Otherwise (default), the first GNSS entry will become the reference.
    std::optional<mrpt::topography::TGeodeticCoords> geodeticReference;

    AddGNSSFactorParams fgParams;

    mrpt::system::COutputLogger* logger = nullptr;
};

/** Function to georeferencing a given SimpleMap with GNSS observations.
 *  A minimum of 3 (non-colinear) KFs with GNSS data are required to solve
 *  for the optimal transformation.
 */
SMGeoReferencingOutput simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm,
    const SMGeoReferencingParams& params = {});

struct FrameGNSS
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

struct GNSSFrames
{
    std::vector<FrameGNSS>                           frames;
    std::optional<mrpt::topography::TGeodeticCoords> refCoord;
};

GNSSFrames extract_gnss_frames_from_sm(
    const mrpt::maps::CSimpleMap&                           sm,
    const std::optional<mrpt::topography::TGeodeticCoords>& refCoord =
        std::nullopt);

void add_gnss_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const GNSSFrames& frames,
    const AddGNSSFactorParams& params);

}  // namespace mola
