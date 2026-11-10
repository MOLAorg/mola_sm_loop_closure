// Unit tests for the GNSS initial alignment fit and the dense trajectory
// correction (no datasets needed).

#include <gtest/gtest.h>
#include <mola_sm_loop_closure/common/densify_trajectory.h>
#include <mola_sm_loop_closure/common/gnss_factor_helpers.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <vector>

using mola::lc_common::StampedPose;
using mrpt::poses::CPose3D;

TEST(GnssAlignment, RecoversYawAndTranslation)
{
    const auto truth = CPose3D::FromXYZYawPitchRoll(12.0, -7.5, 1.25, mrpt::DEG2RAD(117.0), 0, 0);

    std::vector<mrpt::math::TPoint3D> src;
    std::vector<mrpt::math::TPoint3D> dst;
    for (int i = 0; i < 50; i++)
    {
        const double               s = 0.5 * i;
        const mrpt::math::TPoint3D p(s, 0.3 * s + std::sin(s), 0.02 * s);
        src.push_back(p);
        dst.push_back(truth.composePoint(p));
    }
    const auto fit = mola::lc_common::fit_yaw_translation(src, dst, 1.0);
    ASSERT_TRUE(fit.has_value());
    EXPECT_NEAR(fit->yaw(), truth.yaw(), 1e-9);
    EXPECT_NEAR(fit->pitch(), 0.0, 1e-12);
    EXPECT_NEAR(fit->roll(), 0.0, 1e-12);
    EXPECT_NEAR(fit->distanceTo(truth), 0.0, 1e-9);
}

TEST(GnssAlignment, RejectsDegenerateInput)
{
    std::vector<mrpt::math::TPoint3D> src = {{0, 0, 0}, {0.1, 0, 0}, {0, 0.1, 0}, {0.1, 0.1, 0}};
    EXPECT_FALSE(mola::lc_common::fit_yaw_translation(src, src, 1.0).has_value());
    src.resize(2);
    EXPECT_FALSE(mola::lc_common::fit_yaw_translation(src, src, 0.0).has_value());
}

TEST(DensifyTrajectory, AppliesInterpolatedCorrections)
{
    // Dense odometry at 10 Hz along a curve; keyframes every 1 s, stamped 0.03 s
    // earlier than the dense poses (start vs. middle of a scan).
    std::vector<StampedPose> dense;
    for (int i = 0; i <= 100; i++)
    {
        const double t = 100.0 + 0.1 * i;
        dense.push_back(
            {t, CPose3D::FromXYZYawPitchRoll(0.1 * i, 0.002 * i * i, 0, 0.01 * i, 0, 0)});
    }
    const auto correctionOf = [](double t)
    {
        return CPose3D::FromXYZYawPitchRoll(
            5.0 + 0.01 * (t - 100.0), -3.0, 0.5, mrpt::DEG2RAD(30.0 + 0.1 * (t - 100.0)), 0, 0);
    };
    std::vector<StampedPose> kfBefore;
    std::vector<StampedPose> kfAfter;
    for (int i = 0; i <= 100; i += 10)
    {
        const double t = dense[i].stamp - 0.03;
        kfBefore.push_back({t, dense[i].pose});
        kfAfter.push_back({t, correctionOf(dense[i].stamp) + dense[i].pose});
    }

    std::vector<StampedPose> out;
    const auto               r = mola::lc_common::densify_trajectory(kfBefore, kfAfter, dense, out);

    EXPECT_NEAR(r.time_offset, 0.03, 1e-9);
    EXPECT_EQ(r.matched_keyframes, kfBefore.size());
    EXPECT_LT(r.max_error_at_keyframes, 1e-9);
    ASSERT_EQ(out.size(), dense.size());
    for (size_t i = 0; i < dense.size(); i++)
    {
        EXPECT_DOUBLE_EQ(out[i].stamp, dense[i].stamp);
        // The correction varies slowly: interpolation error well below 1 mm.
        const auto expected = correctionOf(dense[i].stamp) + dense[i].pose;
        EXPECT_LT(out[i].pose.distanceTo(expected), 1e-3) << "i=" << i;
    }
}

TEST(DensifyTrajectory, ClampsOutsideTheKeyframeSpan)
{
    std::vector<StampedPose> dense;
    for (int i = 0; i < 30; i++)
    {
        dense.push_back({0.1 * i, CPose3D::FromXYZYawPitchRoll(0.1 * i, 0, 0, 0, 0, 0)});
    }
    const auto               c        = CPose3D::FromXYZYawPitchRoll(1, 2, 3, 0.5, 0, 0);
    std::vector<StampedPose> kfBefore = {{1.0, dense[10].pose}, {2.0, dense[20].pose}};
    std::vector<StampedPose> kfAfter  = {{1.0, c + dense[10].pose}, {2.0, c + dense[20].pose}};

    std::vector<StampedPose> out;
    mola::lc_common::densify_trajectory(kfBefore, kfAfter, dense, out);
    EXPECT_LT(out.front().pose.distanceTo(c + dense.front().pose), 1e-9);
    EXPECT_LT(out.back().pose.distanceTo(c + dense.back().pose), 1e-9);
}

TEST(DensifyTrajectory, RejectsUnsortedInput)
{
    const CPose3D            p;
    std::vector<StampedPose> kf    = {{0.0, p}, {1.0, p}};
    std::vector<StampedPose> dense = {{0.5, p}, {0.2, p}};
    std::vector<StampedPose> out;
    EXPECT_ANY_THROW(mola::lc_common::densify_trajectory(kf, kf, dense, out));
    std::vector<StampedPose> kfUnsorted = {{1.0, p}, {0.0, p}};
    std::vector<StampedPose> denseOk    = {{0.2, p}, {0.5, p}};
    EXPECT_ANY_THROW(mola::lc_common::densify_trajectory(kfUnsorted, kfUnsorted, denseOk, out));
}
