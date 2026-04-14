/* Unit test: invoke mola-sm-lc-cli with the F2F GICP pipeline on the
 * mvsim-warehouse01 simplemap and verify it exits successfully and produces
 * an output file.
 */

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

static std::string getenv_or_empty(const char* name)
{
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
}

TEST(MolaSmLcCli, F2F_GICP_warehouse)
{
    const std::string pipeline  = getenv_or_empty("LC_PIPELINE_YAML");
    const std::string input_sm  = getenv_or_empty("LC_INPUT_SIMPLEMAP");
    const std::string output_sm = getenv_or_empty("LC_OUTPUT_SIMPLEMAP");

    ASSERT_FALSE(pipeline.empty())  << "LC_PIPELINE_YAML env var not set";
    ASSERT_FALSE(input_sm.empty())  << "LC_INPUT_SIMPLEMAP env var not set";
    ASSERT_FALSE(output_sm.empty()) << "LC_OUTPUT_SIMPLEMAP env var not set";

    ASSERT_TRUE(std::filesystem::exists(pipeline))
        << "Pipeline YAML not found: " << pipeline;
    ASSERT_TRUE(std::filesystem::exists(input_sm))
        << "Input simplemap not found: " << input_sm;

    // Remove any leftover output from a previous run
    std::filesystem::remove(output_sm);

    const std::string cmd =
        "mola-sm-lc-cli"
        " -a mola::FrameToFrameLoopClosure"
        " -p \"" + pipeline  + "\""
        " -i \"" + input_sm  + "\""
        " -o \"" + output_sm + "\"";

    const int ret = std::system(cmd.c_str());
    EXPECT_EQ(ret, 0) << "CLI returned non-zero exit code. Command was:\n" << cmd;

    EXPECT_TRUE(std::filesystem::exists(output_sm))
        << "Output simplemap was not created: " << output_sm;
}
