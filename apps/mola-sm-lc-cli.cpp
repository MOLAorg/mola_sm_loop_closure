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

#include <mola_sm_loop_closure/SimplemapLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

// CLI flags:

struct Cli
{
    TCLAP::CmdLine cmd{"mola-sm-lc-cli"};

    TCLAP::ValueArg<std::string> argInput{
        "i",  "input",         "Input .simplemap file",
        true, "map.simplemap", "map.simplemap",
        cmd};

    TCLAP::ValueArg<std::string> argOutput{
        "o",
        "output",
        "Output .simplemap file to write to",
        true,
        "corrected_map.simplemap",
        "corrected_map.simplemap",
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

    TCLAP::ValueArg<std::string> argPipeline{
        "p",
        "pipeline",
        "YAML file with the SimplemapLoopClosure configuration file.",
        true,
        "loop_closure.yaml",
        "loop_closure.yaml",
        cmd};

    TCLAP::ValueArg<std::string> arg_verbosity_level{
        "v",
        "verbosity",
        "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
        false,
        "",
        "INFO",
        cmd};

    TCLAP::ValueArg<std::string> arg_lazy_load_base_dir{
        "",
        "externals-dir",
        "Lazy-load base directory for datasets with externally-stored "
        "observations. If not defined, the program will try anyway to "
        "autodetect any directory side-by-side to the input .simplemap with "
        "the postfix '_Images' and try to use it as lazy-load base directory.",
        false,
        "dataset_Images",
        "<ExternalsDirectory>",
        cmd};
};

void run_sm_to_mm(Cli& cli)
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

    const auto filYaml = cli.argPipeline.getValue();
    ASSERT_FILE_EXISTS_(filYaml);
    auto yamlData = mola::load_yaml_file(filYaml);

    const auto& filSM = cli.argInput.getValue();

    mrpt::maps::CSimpleMap sm;

    std::cout << "[mola-sm-lc-cli] Reading simplemap from: '" << filSM << "'..."
              << std::endl;

    sm.loadFromFile(filSM);

    std::cout << "[mola-sm-lc-cli] Done read simplemap with " << sm.size()
              << " keyframes." << std::endl;
    ASSERT_(!sm.empty());

    mola::SimplemapLoopClosure lc;

    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (cli.arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    }

    lc.setMinLoggingLevel(logLevel);

    lc.initialize(yamlData);

    // try to detect lazy load:
    std::string lazyLoadBaseDir;
    if (cli.arg_lazy_load_base_dir.isSet())
    {  // use provided dir:
        lazyLoadBaseDir = cli.arg_lazy_load_base_dir.getValue();
    }
    else
    {  // try to autodetect:
        auto candidateDir = mrpt::system::pathJoin(
            {mrpt::system::extractFileDirectory(filSM),
             mrpt::system::extractFileName(filSM) + "_Images"});
        if (mrpt::system::directoryExists(candidateDir))
        {
            lazyLoadBaseDir = candidateDir;

            std::cout << "[mola-sm-lc-cli] Found lazy-load base directory: '"
                      << candidateDir << "'\n";
        }
    }

    if (!lazyLoadBaseDir.empty())
        mrpt::io::setLazyLoadPathBase(lazyLoadBaseDir);

    // generate meaningful output debug files, if enabled:
    lc.params_.debug_files_prefix = mrpt::system::extractFileName(filSM);

    // Main stuff here:
    lc.process(sm);

    // save output:
    const auto filOut = cli.argOutput.getValue();
    std::cout << "[mola-sm-lc-cli] Writing output map to: '" << filOut << "'..."
              << std::endl;

    sm.saveToFile(filOut);

    std::cout << "[mola-sm-lc-cli] Done." << std::endl;
}

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv)) return 1;  // should exit.

        run_sm_to_mm(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
