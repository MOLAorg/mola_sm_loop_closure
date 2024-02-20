/* -------------------------------------------------------------------------
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mola_sm_loop_closure/SimplemapLoopClosure.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/system/filesystem.h>

// CLI flags:

struct Cli
{
    TCLAP::CmdLine cmd{"mola-sm-lc-cli"};

    TCLAP::ValueArg<std::string> argInput{
        "i",  "input",         "Input .simplemap file",
        true, "map.simplemap", "map.simplemap",
        cmd};

    TCLAP::ValueArg<std::string> argOutput{
        "o",      "output", "Output .mm file to write to", true, "out.mm",
        "out.mm", cmd};

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
        false,
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
        "observations",
        false,
        "dataset_Images",
        "<ExternalsDirectory>",
        cmd};
};

void run_sm_to_mm(Cli& cli)
{
    const auto& filSM = cli.argInput.getValue();

    mrpt::maps::CSimpleMap sm;

    std::cout << "[sm2mm] Reading simplemap from: '" << filSM << "'..."
              << std::endl;

    sm.loadFromFile(filSM);

    std::cout << "[sm2mm] Done read simplemap with " << sm.size()
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

    {
        const auto filYaml = cli.argPipeline.getValue();
        ASSERT_FILE_EXISTS_(filYaml);
        auto yamlData = mrpt::containers::yaml::FromFile(filYaml);

        lc.initialize(yamlData);
    }

    if (cli.arg_lazy_load_base_dir.isSet())
        mrpt::io::setLazyLoadPathBase(cli.arg_lazy_load_base_dir.getValue());

    // process...
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
