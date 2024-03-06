/* -------------------------------------------------------------------------
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mola_sm_loop_closure/simplemap_georeference.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/system/filesystem.h>

// CLI flags:

struct Cli
{
    TCLAP::CmdLine cmd{"mola-sm-georeferencing-cli"};

    TCLAP::ValueArg<std::string> argInput{
        "i",  "input",         "Input .simplemap file",
        true, "map.simplemap", "map.simplemap",
        cmd};

#if 0
    TCLAP::ValueArg<std::string> argOutput{
        "o",
        "output",
        "Output .simplemap file to write to",
        true,
        "corrected_map.simplemap",
        "corrected_map.simplemap",
        cmd};
#endif

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
    const auto& filSM = cli.argInput.getValue();

    mrpt::maps::CSimpleMap sm;

    std::cout << "[mola-sm-georeferencing-cli] Reading simplemap from: '"
              << filSM << "'..." << std::endl;

    sm.loadFromFile(filSM);

    std::cout << "[mola-sm-georeferencing-cli] Done read simplemap with "
              << sm.size() << " keyframes." << std::endl;
    ASSERT_(!sm.empty());

    mola::SMGeoReferencingParams p;

    const mola::SMGeoReferencingOutput smGeo =
        mola::simplemap_georeference(sm, p);
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
