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

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/topography/conversions.h>

#include <fstream>

// CLI flags:
struct Cli
{
    TCLAP::CmdLine cmd{"mola-trajectory-georef-cli"};

    TCLAP::ValueArg<std::string> argMM{
        "m",      "map", "Input .mm map with georef info", true, "map.mm",
        "map.mm", cmd};

    TCLAP::ValueArg<std::string> argTraj{
        "t",  "trajectory", "Input .tum trajectory, in map local coordinates",
        true, "traj.tum",   "traj.tum",
        cmd};

    TCLAP::ValueArg<std::string> argOutKML{
        "o",  "output",   "The name of the google earth kml file to write to",
        true, "path.kml", "path.kml",
        cmd};
};

void run_traj_georef(Cli& cli)
{
    const auto filMM = cli.argMM.getValue();

    mp2p_icp::metric_map_t mm;

    std::cout << "[mola-trajectory-georef-cli] Reading input map from: '"
              << filMM << "'..." << std::endl;

    mm.load_from_file(filMM);

    std::cout << "[mola-trajectory-georef-cli] Done read map: "
              << mm.contents_summary() << std::endl;

    ASSERT_(mm.georeferencing.has_value());
    const auto& geo = *mm.georeferencing;

    // trajectory:
    mrpt::poses::CPose3DInterpolator traj;
    bool trajLoadOk = traj.loadFromTextFile_TUM(cli.argTraj.getValue());
    ASSERT_(trajLoadOk);

    const auto WGS84 = mrpt::topography::TEllipsoid::Ellipsoid_WGS84();

    std::vector<mrpt::topography::TGeodeticCoords> geoPath;

    for (const auto& [t, p] : traj)
    {
        const auto enu =
            (geo.T_enu_to_map.mean + mrpt::poses::CPose3D(p)).translation();

        mrpt::topography::TGeocentricCoords gcPt;
        mrpt::topography::ENUToGeocentric(enu, geo.geo_coord, gcPt, WGS84);

        mrpt::topography::TGeodeticCoords ptCoords;
        mrpt::topography::geocentricToGeodetic(gcPt, ptCoords, WGS84);

        geoPath.push_back(ptCoords);
    }

    // Generate KML:
    std::ofstream f(cli.argOutKML.getValue());
    ASSERT_(f.is_open());

    f << mrpt::format(
        R"xml(<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark>
      <name>%s</name>
      <LineString>
        <tessellate>1</tessellate>
        <coordinates>)xml",
        mrpt::system::extractFileName(cli.argTraj.getValue()).c_str());

    for (const auto& d : geoPath)
    {
        f << mrpt::format(
            "%f,%f,0\n", d.lon.decimal_value, d.lat.decimal_value);
    }

    f << R"xml(
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
)xml";
}

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv)) return 1;  // should exit.

        run_traj_georef(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
