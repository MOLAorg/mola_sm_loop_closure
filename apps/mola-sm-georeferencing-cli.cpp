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

#include <mola_sm_loop_closure/simplemap_georeference.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

// CLI flags:

struct Cli
{
    TCLAP::CmdLine cmd{"mola-sm-georeferencing-cli"};

    TCLAP::ValueArg<std::string> argInput{
        "i",  "input",         "Input .simplemap file",
        true, "map.simplemap", "map.simplemap",
        cmd};

    TCLAP::ValueArg<std::string> argWriteMMInto{
        "",
        "write-into",
        "An existing .mm file in which to write the georeferencing metadata",
        false,
        "map.mm",
        "map.mm",
        cmd};

    TCLAP::ValueArg<double> argHorz{
        "",
        "horizontality-sigma",
        "For short trajectories (not >10x the GPS uncertainty), this helps to "
        "avoid degeneracy.",
        false,
        1.0,
        "1.0",
        cmd};

    TCLAP::ValueArg<std::string> argPlugins{
        "l",
        "load-plugins",
        "One or more (comma separated) *.so files to load as plugins, e.g. "
        "defining new CMetricMap classes",
        false,
        "foobar.so",
        "foobar.so",
        cmd};

    TCLAP::ValueArg<std::string> arg_verbosity_level{
        "v",
        "verbosity",
        "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
        false,
        "",
        "INFO",
        cmd};
};

void run_sm_georef(Cli& cli)
{
    if (cli.argPlugins.isSet())
    {
        std::string sErrs;
        bool        ok =
            mrpt::system::loadPluginModules(cli.argPlugins.getValue(), sErrs);
        if (!ok)
        {
            std::cerr << "Errors loading plugins: " << cli.argPlugins.getValue()
                      << std::endl;
            throw std::runtime_error(sErrs.c_str());
        }
    }

    const auto& filSM = cli.argInput.getValue();

    mrpt::maps::CSimpleMap sm;

    std::cout << "[mola-sm-georeferencing-cli] Reading simplemap from: '"
              << filSM << "'..." << std::endl;

    sm.loadFromFile(filSM);

    std::cout << "[mola-sm-georeferencing-cli] Done read simplemap with "
              << sm.size() << " keyframes." << std::endl;
    ASSERT_(!sm.empty());

    mola::SMGeoReferencingParams p;

    if (cli.argHorz.isSet())
    {
        p.fgParams.addHorizontalityConstraints = true;
        p.fgParams.horizontalitySigmaZ         = cli.argHorz.getValue();
    }

    const mola::SMGeoReferencingOutput smGeo =
        mola::simplemap_georeference(sm, p);

    std::cout << "Obtained georeferencing:\n"
              << "lat: " << smGeo.geo_ref.geo_coord.lat.getAsString() << "\n"
              << "lon: " << smGeo.geo_ref.geo_coord.lon.getAsString() << "\n"
              << mrpt::format(
                     "lat_lon: %.06f, %.06f\n",
                     smGeo.geo_ref.geo_coord.lat.decimal_value,
                     smGeo.geo_ref.geo_coord.lon.decimal_value)
              << "h: " << smGeo.geo_ref.geo_coord.height << "\n"
              << "T_enu_to_map: " << smGeo.geo_ref.T_enu_to_map.asString()
              << "\n";

    if (cli.argWriteMMInto.isSet())
    {
        mp2p_icp::metric_map_t mm;

        std::cout << "[mola-sm-georeferencing-cli] Loading mm map: '"
                  << cli.argWriteMMInto.getValue() << "'..." << std::endl;

        mm.load_from_file(cli.argWriteMMInto.getValue());

        // overwrite metadata:
        mm.georeferencing = smGeo.geo_ref;

        // and save:
        mm.save_to_file(cli.argWriteMMInto.getValue());

        std::cout << "[mola-sm-georeferencing-cli] Writing modified mm map: '"
                  << cli.argWriteMMInto.getValue() << "'..." << std::endl;
    }
}

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv)) return 1;  // should exit.

        run_sm_georef(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
