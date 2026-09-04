/* Unit test: FrameToFrameLoopClosure candidate admission and edge correctness.
 * A revisit is a return to the same place, so it must be proposable at nearly
 * zero separation, and on a near-truth input map the edge it yields must agree
 * with what that map already says.
 */

#include <gtest/gtest.h>
#include <mola_sm_loop_closure/FrameToFrameLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SO.h>

#include <cstdlib>
#include <filesystem>
#include <string>

namespace
{
/** The named environment variable, or an empty string when it is unset. */
std::string getenv_or_empty(const char* name)
{
    const char* v = std::getenv(name);
    return v != nullptr ? std::string(v) : std::string();
}

/** Mean pose of one keyframe of the map. */
mrpt::poses::CPose3D kf_pose(const mrpt::maps::CSimpleMap& sm, uint32_t i)
{
    return sm.get(i).pose->getMeanVal();
}

/** Separation of the two keyframes an edge joins, as the input map has them. */
double map_separation(const mrpt::maps::CSimpleMap& sm, uint32_t i, uint32_t j)
{
    return (kf_pose(sm, i).translation() - kf_pose(sm, j).translation()).norm();
}

// The warehouse fixture is simulated, and its trajectory returns to within
// half a metre of itself, so its own relative poses are close enough to truth
// to check an edge against. These bounds are wide enough not to be brittle
// about a genuine correction and narrow enough to catch a registration that
// has locked onto the wrong place, which is failure mode this guards.
constexpr double MAX_EDGE_TRANSLATION_ERROR = 0.5;  // [m]
constexpr double MAX_EDGE_ROTATION_ERROR    = 5.0;  // [deg]

// A pair this close together in the map is a revisit by any reading.
constexpr double REVISIT_SEPARATION = 1.5;  // [m]
}  // namespace

TEST(MolaSmLcCandidates, default_candidate_floor_admits_revisits)
{
    // The candidate distance floor is measured in the current estimate, so any
    // non-zero value rejects revisits outright on an accurate odometry. Pinned
    // here because it is a contract of the class, not a tuning choice.
    const mola::FrameToFrameLoopClosure::Parameters defaults;
    EXPECT_EQ(defaults.min_distance_between_frames, 0.0);
}

TEST(MolaSmLcCandidates, F2F_warehouse_revisits_are_proposed)
{
    const std::string pipeline = getenv_or_empty("LC_PIPELINE_YAML");
    const std::string input_sm = getenv_or_empty("LC_INPUT_SIMPLEMAP");

    ASSERT_FALSE(pipeline.empty()) << "LC_PIPELINE_YAML env var not set";
    ASSERT_FALSE(input_sm.empty()) << "LC_INPUT_SIMPLEMAP env var not set";
    ASSERT_TRUE(std::filesystem::exists(pipeline)) << "Pipeline YAML not found: " << pipeline;
    ASSERT_TRUE(std::filesystem::exists(input_sm)) << "Input simplemap not found: " << input_sm;

    mrpt::maps::CSimpleMap sm;
    ASSERT_TRUE(sm.loadFromFile(input_sm)) << "Failed to load simplemap: " << input_sm;
    ASSERT_GT(sm.size(), 0U);

    // Clear the candidate-floor hook before loading, so what is exercised is
    // the pipeline's own default and not whatever the environment supplies.
    ::unsetenv("MIN_LC_DISTANCE");

    mola::FrameToFrameLoopClosure lc;
    auto                          cfg = mola::load_yaml_file(pipeline);
    if (cfg.has("params"))
    {
        // No debug artifacts on disk. Everything else, and in particular the
        // acceptance gate, is left at the pipeline's production values: what is
        // under test is that a real loop survives them.
        cfg["params"]["save_trajectory_files"] = false;
        cfg["params"]["save_3d_scene_files"]   = false;
    }
    lc.initialize(cfg);

    const auto edges = lc.analyze(sm);

    // The fixture revisits its own trajectory, so the production gate must
    // accept something. It accepts nothing whenever the candidate search
    // imposes a spatial floor, because that floor discards revisits first.
    ASSERT_GE(edges.size(), 1U) << "no loop closure accepted on a map that revisits itself";

    size_t nRevisits = 0;
    for (const auto& e : edges)
    {
        ASSERT_LT(e.from, sm.size());
        ASSERT_LT(e.to, sm.size());

        const double sep = map_separation(sm, e.from, e.to);
        if (sep < REVISIT_SEPARATION)
        {
            nRevisits++;
        }

        // pose_to = pose_from (+) relative_pose, so the map's own answer for
        // the same quantity is the inverse composition of the two keyframes.
        const auto relMap = kf_pose(sm, e.to) - kf_pose(sm, e.from);
        const auto err    = e.relative_pose.mean - relMap;

        const double errXyz = err.translation().norm();
        const double errDeg =
            mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(err.getRotationMatrix()).norm());

        EXPECT_LT(errXyz, MAX_EDGE_TRANSLATION_ERROR)
            << "edge " << e.from << " <-> " << e.to << " (separated by " << sep
            << " m in the map) disagrees with it by " << errXyz << " m";
        EXPECT_LT(errDeg, MAX_EDGE_ROTATION_ERROR)
            << "edge " << e.from << " <-> " << e.to << " disagrees with the map by " << errDeg
            << " deg";
    }

    // The point of the search is to find where the robot came back. If every
    // accepted edge joins two places the robot merely passed near, the
    // candidate window is looking in the wrong range.
    EXPECT_GE(nRevisits, 1U) << "no accepted edge joins a pair closer than " << REVISIT_SEPARATION
                             << " m; the candidate window admits no revisits";
}
