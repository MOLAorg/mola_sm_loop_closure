// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
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

#include <mola_sm_loop_closure/LoopClosureInterface.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>

namespace
{

// CLI flags:
CLI::App cmd{"mola-sm-lc-cli"};

std::string argInput  = "map.simplemap";
std::string argOutput = "corrected_map.simplemap";
std::string argPlugins;
std::string argPipeline         = "loop_closure.yaml";
std::string arg_verbosity_level = "INFO";
std::string arg_algo            = "mola::SimplemapLoopClosure";
std::string arg_lazy_load_base_dir;

CLI::Option* optPlugins;
CLI::Option* optVerbosityLevel;
CLI::Option* optLazyLoadBaseDir;

void run_sm_to_mm()
{
    if (optPlugins->count() > 0)
    {
        std::string sErrs;
        bool        ok = mrpt::system::loadPluginModules(argPlugins, sErrs);
        if (!ok)
        {
            std::cerr << "Errors loading plugins: " << argPlugins << "\n";
            throw std::runtime_error(sErrs.c_str());
        }
    }

    const auto& filYaml = argPipeline;
    ASSERT_FILE_EXISTS_(filYaml);
    auto yamlData = mola::load_yaml_file(filYaml);

    const auto& filSM  = argInput;
    const auto& filOut = argOutput;

    mrpt::maps::CSimpleMap sm;

    std::cout << "[mola-sm-lc-cli] Reading simplemap from: '" << filSM << "'...\n";

    bool loadOk = sm.loadFromFile(filSM);
    ASSERT_(loadOk);

    std::cout << "[mola-sm-lc-cli] Done read simplemap with " << sm.size() << " keyframes.\n";
    ASSERT_(!sm.empty());

    // Create algorithm:
    auto algoPtr = mrpt::rtti::classFactory(arg_algo);
    if (!algoPtr)
    {
        THROW_EXCEPTION_FMT("Unregistered algorithm C++ class: '%s'", arg_algo.c_str());
    }
    auto lcPtr = std::dynamic_pointer_cast<mola::LoopClosureInterface>(algoPtr);
    if (!lcPtr)
    {
        THROW_EXCEPTION_FMT(
            "Algorithm C++ class seems not to be an implementation of 'LoopClosureInterface': '%s'",
            arg_algo.c_str());
    }
    auto& lc = *lcPtr;

    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (optVerbosityLevel->count() > 0)
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(arg_verbosity_level);
    }

    // Set "params.debug_files_prefix" so generated .tum files, etc. have the expected prefix:
    if (yamlData.has("params"))
    {
        auto debugFilesPrefix = mrpt::system::pathJoin(
            {mrpt::system::extractFileDirectory(filOut),
             mrpt::system::extractFileName(filOut) + "_lc_"});

        yamlData["params"]["debug_files_prefix"] = debugFilesPrefix;
    }

    lc.setMinLoggingLevel(logLevel);

    lc.initialize(yamlData);

    // try to detect lazy load:
    std::string lazyLoadBaseDir;
    if (optLazyLoadBaseDir->count() > 0)
    {  // use provided dir:
        lazyLoadBaseDir = arg_lazy_load_base_dir;
    }
    else
    {  // try to autodetect:
        auto candidateDir = mrpt::system::pathJoin(
            {mrpt::system::extractFileDirectory(filSM),
             mrpt::system::extractFileName(filSM) + "_Images"});
        if (mrpt::system::directoryExists(candidateDir))
        {
            lazyLoadBaseDir = candidateDir;

            std::cout << "[mola-sm-lc-cli] Found lazy-load base directory: '" << candidateDir
                      << "'\n";
        }
    }

    if (!lazyLoadBaseDir.empty())
    {
        mrpt::io::setLazyLoadPathBase(lazyLoadBaseDir);
    }

    // Main stuff here:
    lc.process(sm);

    // save output:
    std::cout << "[mola-sm-lc-cli] Writing output map to: '" << filOut << "'...\n";

    bool saveOk = sm.saveToFile(filOut);
    ASSERT_(saveOk);

    std::cout << "[mola-sm-lc-cli] Done.\n";
}
}  // namespace

int main(int argc, char** argv)
{
    cmd.add_option("-i,--input", argInput, "Input .simplemap file")->required();

    cmd.add_option("-o,--output", argOutput, "Output .simplemap file to write to")->required();

    optPlugins = cmd.add_option(
        "-l,--load-plugins", argPlugins,
        "One or more (comma separated) *.so files to load as plugins, e.g. "
        "defining new CMetricMap classes");

    cmd.add_option(
           "-p,--pipeline", argPipeline,
           "YAML file with the loop closure algorithm configuration file.")
        ->required();

    optVerbosityLevel = cmd.add_option(
        "-v,--verbosity", arg_verbosity_level,
        "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");

    cmd.add_option(
        "-a,--algorithm", arg_algo, "C++ class name of the loop-closure algorithm to use.");

    optLazyLoadBaseDir = cmd.add_option(
        "--externals-dir", arg_lazy_load_base_dir,
        "Lazy-load base directory for datasets with externally-stored "
        "observations. If not defined, the program will try anyway to "
        "autodetect any directory side-by-side to the input .simplemap with "
        "the postfix '_Images' and try to use it as lazy-load base directory.");

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        run_sm_to_mm();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
