// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
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

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/topography/conversions.h>

#include <fstream>

// CLI flags:
struct Cli
{
    TCLAP::CmdLine cmd{"mola-trajectory-georef-cli"};

    TCLAP::ValueArg<std::string> argMM{
        "m",      "map", "Input .mm map with georef info", false, "map.mm",
        "map.mm", cmd};

    TCLAP::ValueArg<std::string> argGeoRef{
        "g",   "geo-ref",    "Input .georef file with georef info",
        false, "map.georef", "map.georef",
        cmd};

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
    std::optional<mp2p_icp::metric_map_t::Georeferencing> geo;

    if (cli.argMM.isSet())
    {
        const auto filMM = cli.argMM.getValue();

        mp2p_icp::metric_map_t mm;

        std::cout << "[mola-trajectory-georef-cli] Reading input map from: '"
                  << filMM << "'..." << std::endl;

        mm.load_from_file(filMM);

        std::cout << "[mola-trajectory-georef-cli] Done read map: "
                  << mm.contents_summary() << std::endl;

        ASSERT_(mm.georeferencing.has_value());
        geo = mm.georeferencing;
    }
    else if (cli.argGeoRef.isSet())
    {
        mrpt::io::CFileGZInputStream f(cli.argGeoRef.getValue());

        auto arch = mrpt::serialization::archiveFrom(f);
        arch >> geo;
    }
    else
    {
        THROW_EXCEPTION(
            "Missing cli argument: at least one source of geo-referencing must "
            "be given");
    }

    ASSERT_(geo.has_value());

    // trajectory:
    mrpt::poses::CPose3DInterpolator traj;
    bool trajLoadOk = traj.loadFromTextFile_TUM(cli.argTraj.getValue());
    ASSERT_(trajLoadOk);

    const auto WGS84 = mrpt::topography::TEllipsoid::Ellipsoid_WGS84();

    std::vector<mrpt::topography::TGeodeticCoords> geoPath;

    for (const auto& [t, p] : traj)
    {
        const auto enu =
            (geo->T_enu_to_map.mean + mrpt::poses::CPose3D(p)).translation();

        mrpt::topography::TGeocentricCoords gcPt;
        mrpt::topography::ENUToGeocentric(enu, geo->geo_coord, gcPt, WGS84);

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
