/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

#include <gtest/gtest.h>
#include <mola_sm_loop_closure/global_registration/bev_correlation.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace
{
using mola::global_registration::bev_correlation;
using mola::global_registration::BevCorrelationParameters;
using mrpt::poses::CPose3D;

/** Random walls and boxes of different heights, plus flat ground, in a world
 *  frame with z up. */
std::vector<mrpt::math::TPoint3D> make_scene(unsigned int seed)
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(seed);
    std::vector<mrpt::math::TPoint3D> pts;
    for (double x = -60; x <= 60; x += 1.0)
    {
        for (double y = -60; y <= 60; y += 1.0)
        {
            pts.emplace_back(x, y, 0);
        }
    }
    for (int b = 0; b < 40; b++)
    {
        const double cx  = rng.drawUniform(-55, 55);
        const double cy  = rng.drawUniform(-55, 55);
        const double lx  = rng.drawUniform(1, 12);
        const double ly  = rng.drawUniform(0.3, 4);
        const double h   = rng.drawUniform(0.5, 6);
        const double yaw = rng.drawUniform(0, M_PI);
        for (double u = -lx / 2; u <= lx / 2; u += 0.2)
        {
            for (double v = -ly / 2; v <= ly / 2; v += 0.2)
            {
                for (double z = 0.2; z <= h; z += 0.25)
                {
                    pts.emplace_back(
                        cx + std::cos(yaw) * u - std::sin(yaw) * v,
                        cy + std::sin(yaw) * u + std::cos(yaw) * v, z);
                }
            }
        }
    }
    return pts;
}

/** The scene points within `range` of the frame origin, in that frame. */
mrpt::maps::CSimplePointsMap observe(
    const std::vector<mrpt::math::TPoint3D>& scene, const CPose3D& frame, double range)
{
    mrpt::maps::CSimplePointsMap pc;
    for (const auto& p : scene)
    {
        if (std::hypot(p.x - frame.x(), p.y - frame.y()) > range)
        {
            continue;
        }
        const auto q = frame.inverseComposePoint(p);
        pc.insertPoint(q);
    }
    return pc;
}

double rot_error_deg(const CPose3D& a, const CPose3D& b)
{
    return mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log((a - b).getRotationMatrix()).norm());
}

}  // namespace

TEST(BevCorrelation, SyntheticLargeYawAndOffset)
{
    const auto scene = make_scene(123);
    // Tilted frames: the attitudes give roll and pitch, as gravity-aligned odometry would.
    const auto gFrame =
        CPose3D::FromXYZYawPitchRoll(0, 0, 1.0, 0.3, mrpt::DEG2RAD(2.0), mrpt::DEG2RAD(-3.0));
    const auto lFrame = CPose3D::FromXYZYawPitchRoll(
        14.0, -9.0, 1.4, mrpt::DEG2RAD(160.0), mrpt::DEG2RAD(-1.5), mrpt::DEG2RAD(2.5));
    const auto global = observe(scene, gFrame, 50);
    const auto local  = observe(scene, lFrame, 50);

    const BevCorrelationParameters params;
    const auto                     r     = bev_correlation(global, gFrame, local, lFrame, params);
    const auto                     truth = lFrame - gFrame;
    std::cout << "truth: " << truth << "\nfound: " << r.pose << "\nscore: " << r.score
              << " ambiguity ratio: " << r.ambiguity_ratio << "\n";

    EXPECT_TRUE(r.valid);
    EXPECT_LT((r.pose.translation() - truth.translation()).norm(), 2.0);
    EXPECT_LT(rot_error_deg(r.pose, truth), params.yaw_step_deg + 1.0);
}

TEST(BevCorrelation, UnrelatedScenesAreRejected)
{
    const auto    a = make_scene(1);
    const auto    b = make_scene(2);
    const CPose3D frame(0, 0, 1.0, 0, 0, 0);
    const auto r = bev_correlation(observe(a, frame, 50), frame, observe(b, frame, 50), frame, {});
    std::cout << "score: " << r.score << " ambiguity ratio: " << r.ambiguity_ratio << "\n";
    EXPECT_FALSE(r.valid);
}

// Loop closure cases of mola_test_datasets (datasets/loop_closures/*/cases.yaml):
// every set found there is used. Per set, (almost) all loops must be found, and
// every valid result must be right: wrong matches and non-loops must be
// rejected (or, for a wrong match, found at the true pose).
TEST(BevCorrelation, DatasetLoopClosureCases)
{
    const char*                        dir = std::getenv("LC_CASES_DIR");
    std::vector<std::filesystem::path> sets;
    if (dir && std::filesystem::is_directory(dir))
    {
        for (const auto& e : std::filesystem::directory_iterator(dir))
        {
            if (e.is_directory() && std::filesystem::exists(e.path() / "cases.yaml"))
            {
                sets.push_back(e.path());
            }
        }
    }
    if (sets.empty())
    {
        GTEST_SKIP() << "No loop closure cases found (LC_CASES_DIR)";
    }
    std::sort(sets.begin(), sets.end());

    // Minimum fraction of loops found per set. Sets with cases known to be
    // beyond the method have a lower floor, to catch regressions: husky (sparse
    // scans far apart), kitti (a repetitive road, a far pair with little overlap).
    const std::map<std::string, double> minFoundPerSet  = {{"husky", 0.25}, {"kitti", 0.9}};
    const double                        minFoundDefault = 0.95;

    const auto loadPoints = [&](const std::string& f)
    {
        mp2p_icp::metric_map_t mm;
        const bool             ok = mm.load_from_file(std::string(dir) + "/" + f);
        EXPECT_TRUE(ok) << f;
        return mm.point_layer("points");
    };
    const auto pose = [](const mrpt::containers::yaml& v)
    {
        return CPose3D::FromXYZYawPitchRoll(
            v(0).as<double>(), v(1).as<double>(), v(2).as<double>(),
            mrpt::DEG2RAD(v(3).as<double>()), mrpt::DEG2RAD(v(4).as<double>()),
            mrpt::DEG2RAD(v(5).as<double>()));
    };

    const BevCorrelationParameters params;
    for (const auto& set : sets)
    {
        const auto setName = set.filename().string();
        int        nLoops  = 0;
        int        nFound  = 0;
        const auto doc     = mrpt::containers::yaml::FromFile((set / "cases.yaml").string());
        for (const auto& node : doc["cases"].asSequence())
        {
            const auto c    = mrpt::containers::yaml(node);
            const auto name = c["name"].as<std::string>();
            const auto kind = c["kind"].as<std::string>();
            const auto g    = loadPoints(c["global"].as<std::string>());
            const auto l    = loadPoints(c["local"].as<std::string>());
            const auto r    = bev_correlation(
                   *g, pose(c["global_odometry_pose"]), *l, pose(c["local_odometry_pose"]), params);
            const auto   ref     = pose(c["reference_pose"]);
            const double et      = (r.pose.translation() - ref.translation()).norm();
            const double er      = rot_error_deg(r.pose, ref);
            const bool   correct = et < 2.0 && er < 5.0;
            std::cout << name << " " << kind << " valid=" << r.valid << " score=" << r.score
                      << " ratio=" << r.ambiguity_ratio << " err=" << et << " m " << er << " deg\n";
            if (kind == "loop")
            {
                nLoops++;
                nFound += (r.valid && correct) ? 1 : 0;
            }
            else if (kind == "no_loop")
            {
                EXPECT_FALSE(r.valid) << name << ": a non-loop was accepted";
            }
            EXPECT_FALSE(r.valid && !correct) << name << ": valid but wrong result";
        }
        const auto   it      = minFoundPerSet.find(setName);
        const double minFrac = it != minFoundPerSet.end() ? it->second : minFoundDefault;
        std::cout << setName << ": loops found " << nFound << "/" << nLoops << " (minimum "
                  << minFrac << ")\n";
        EXPECT_GE(nFound, minFrac * nLoops) << setName;
    }
}
