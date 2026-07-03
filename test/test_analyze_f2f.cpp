/* Unit test: FrameToFrameLoopClosure::analyze() on the mvsim-warehouse01
 * simplemap. Verifies the detector-only path returns well-formed proposed
 * edges and does NOT mutate the input map.
 */

#include <gtest/gtest.h>
#include <mola_sm_loop_closure/FrameToFrameLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/maps/CSimpleMap.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>

static std::string getenv_or_empty(const char* name)
{
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
}

TEST(MolaSmLcAnalyze, F2F_warehouse)
{
    const std::string pipeline = getenv_or_empty("LC_PIPELINE_YAML");
    const std::string input_sm = getenv_or_empty("LC_INPUT_SIMPLEMAP");

    ASSERT_FALSE(pipeline.empty()) << "LC_PIPELINE_YAML env var not set";
    ASSERT_FALSE(input_sm.empty()) << "LC_INPUT_SIMPLEMAP env var not set";
    ASSERT_TRUE(std::filesystem::exists(pipeline)) << "Pipeline YAML not found: " << pipeline;
    ASSERT_TRUE(std::filesystem::exists(input_sm)) << "Input simplemap not found: " << input_sm;

    mrpt::maps::CSimpleMap sm;
    ASSERT_TRUE(sm.loadFromFile(input_sm)) << "Failed to load simplemap: " << input_sm;
    const size_t framesBefore = sm.size();
    ASSERT_GT(framesBefore, 0U);

    mola::FrameToFrameLoopClosure lc;
    auto                          cfg = mola::load_yaml_file(pipeline);
    if (cfg.has("params"))
    {
        // No debug artifacts on disk.
        cfg["params"]["save_trajectory_files"] = false;
        cfg["params"]["save_3d_scene_files"]   = false;
        // Accept any candidate that yields an ICP result so the edge-building
        // path is exercised: this dataset has valid loop-closure candidates but
        // their ICP goodness sits below the production threshold, so a strict
        // gate would accept none and leave nothing to check.
        cfg["params"]["min_icp_goodness"] = 0.0;
    }
    lc.initialize(cfg);

    const auto edges = lc.analyze(sm);

    // Detector-only: the input map must be untouched.
    EXPECT_EQ(sm.size(), framesBefore) << "analyze() must not mutate the input map";

    // The warehouse sequence has loop-closure candidates; with the gate opened
    // above, analyze() must return at least one well-formed edge.
    EXPECT_GE(edges.size(), 1U);

    for (const auto& e : edges)
    {
        EXPECT_LT(e.from, framesBefore);
        EXPECT_LT(e.to, framesBefore);
        EXPECT_NE(e.from, e.to);
        EXPECT_GE(e.quality, 0.0);
        EXPECT_LE(e.quality, 1.0);
        // Covariance diagonal must be finite and positive.
        for (int i = 0; i < 6; i++)
        {
            const double var = e.relative_pose.cov(i, i);
            EXPECT_TRUE(std::isfinite(var)) << "non-finite covariance at " << i;
            EXPECT_GT(var, 0.0) << "non-positive covariance at " << i;
        }
    }
}
