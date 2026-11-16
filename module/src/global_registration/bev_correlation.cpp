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

#include <mola_sm_loop_closure/global_registration/bev_correlation.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/fourier.h>
#include <mrpt/math/wrap2pi.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace mola::global_registration
{
void BevCorrelationParameters::load_from_yaml(const mrpt::containers::yaml& cfg)
{
    YAML_LOAD_OPT(grid_resolution, double);
    YAML_LOAD_OPT(max_range, double);
    YAML_LOAD_OPT(yaw_step_deg, double);
    YAML_LOAD_OPT(max_cell_height, double);
    YAML_LOAD_OPT(min_score, double);
    YAML_LOAD_OPT(min_ambiguity_ratio, double);
    YAML_LOAD_OPT(ambiguity_min_distance, double);
    YAML_LOAD_OPT(ambiguity_min_yaw_deg, double);
}

namespace
{
using mrpt::math::CMatrixFloat;

/** Leveled points: a cloud rotated by the roll and pitch of its attitude. */
struct Points
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
};

Points level(const mrpt::maps::CPointsMap& pc, const mrpt::poses::CPose3D& attitude)
{
    const auto R = mrpt::poses::CPose3D::FromYawPitchRoll(0, attitude.pitch(), attitude.roll())
                       .getRotationMatrix();
    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();
    Points      out;
    out.x.resize(xs.size());
    out.y.resize(xs.size());
    out.z.resize(xs.size());
    for (size_t i = 0; i < xs.size(); i++)
    {
        out.x[i] = static_cast<float>(R(0, 0) * xs[i] + R(0, 1) * ys[i] + R(0, 2) * zs[i]);
        out.y[i] = static_cast<float>(R(1, 0) * xs[i] + R(1, 1) * ys[i] + R(1, 2) * zs[i]);
        out.z[i] = static_cast<float>(R(2, 0) * xs[i] + R(2, 1) * ys[i] + R(2, 2) * zs[i]);
    }
    return out;
}

/** A square bird's eye view grid of N x N cells, centered at the origin. */
struct Grid
{
    size_t             N   = 0;
    double             res = 1;
    std::vector<float> zmin;
    std::vector<float> zmax;

    Grid(size_t n, double resolution)
        : N(n),
          res(resolution),
          zmin(n * n, std::numeric_limits<float>::max()),
          zmax(n * n, std::numeric_limits<float>::lowest())
    {
    }

    [[nodiscard]] bool occupied(size_t k) const { return zmax[k] >= zmin[k]; }

    /// Inserts the points rotated by yaw (cos, sin) and shifted by (dx, dy, dz).
    void insert(const Points& p, double c, double s, double dx, double dy, double dz)
    {
        const double half = 0.5 * static_cast<double>(N) * res;
        for (size_t i = 0; i < p.x.size(); i++)
        {
            const double x  = c * p.x[i] - s * p.y[i] + dx;
            const double y  = s * p.x[i] + c * p.y[i] + dy;
            const auto   ix = static_cast<long>(std::floor((x + half) / res));
            const auto   iy = static_cast<long>(std::floor((y + half) / res));
            if (ix < 0 || iy < 0 || ix >= static_cast<long>(N) || iy >= static_cast<long>(N))
            {
                continue;
            }
            const size_t k = static_cast<size_t>(ix) * N + static_cast<size_t>(iy);
            const auto   z = static_cast<float>(p.z[i] + dz);
            zmin[k]        = std::min(zmin[k], z);
            zmax[k]        = std::max(zmax[k], z);
        }
    }

    /** Writes the zero-mean (over occupied cells) clamped vertical extent into
     *  the top-left N x N block of an M x M zero matrix. Returns its norm. */
    double to_matrix(double maxHeight, CMatrixFloat& m, size_t M) const
    {
        m.setZero(static_cast<int>(M), static_cast<int>(M));
        double sum = 0;
        size_t n   = 0;
        for (size_t k = 0; k < N * N; k++)
        {
            if (occupied(k))
            {
                sum += std::min<double>(zmax[k] - zmin[k], maxHeight);
                n++;
            }
        }
        if (n == 0)
        {
            return 0;
        }
        const double mean  = sum / static_cast<double>(n);
        double       norm2 = 0;
        for (size_t k = 0; k < N * N; k++)
        {
            if (!occupied(k))
            {
                continue;
            }
            const double v = std::min<double>(zmax[k] - zmin[k], maxHeight) - mean;
            m(static_cast<int>(k / N), static_cast<int>(k % N)) = static_cast<float>(v);
            norm2 += v * v;
        }
        return std::sqrt(norm2);
    }
};

size_t next_pow2(size_t n)
{
    size_t p = 1;
    while (p < n)
    {
        p <<= 1;
    }
    return p;
}

/** A correlation peak: score, yaw [rad], and translation [m]. */
struct Peak
{
    double score = -1;
    double yaw   = 0;
    double dx    = 0;
    double dy    = 0;
};

}  // namespace

BevCorrelationResult bev_correlation(
    const mrpt::maps::CPointsMap& global, const mrpt::poses::CPose3D& globalAttitude,
    const mrpt::maps::CPointsMap& local, const mrpt::poses::CPose3D& localAttitude,
    const BevCorrelationParameters& params)
{
    BevCorrelationResult result;
    ASSERT_GT_(params.grid_resolution, 0);
    ASSERT_GT_(params.yaw_step_deg, 0);

    const Points pg = level(global, globalAttitude);
    const Points pl = level(local, localAttitude);

    const double res = params.grid_resolution;
    const size_t N   = next_pow2(static_cast<size_t>(std::ceil(2 * params.max_range / res)));
    const size_t M   = 2 * N;  // zero padding: linear, not circular, correlation

    Grid gGrid(N, res);
    gGrid.insert(pg, 1, 0, 0, 0, 0);
    CMatrixFloat gMat;
    const double gNorm = gGrid.to_matrix(params.max_cell_height, gMat, M);
    if (gNorm <= 0)
    {
        return result;
    }
    CMatrixFloat gRe;
    CMatrixFloat gIm;
    mrpt::math::dft2_real(gMat, gRe, gIm);

    // Scale of the inverse transform, from the autocorrelation at zero shift:
    CMatrixFloat pRe(static_cast<int>(M), static_cast<int>(M));
    CMatrixFloat pIm(static_cast<int>(M), static_cast<int>(M));
    CMatrixFloat corr;
    pRe.asEigen() = gRe.asEigen().array().square() + gIm.asEigen().array().square();
    pIm.setZero();
    mrpt::math::idft2_real(pRe, pIm, corr);
    const double invScale = (gNorm * gNorm) / static_cast<double>(corr(0, 0));

    const auto Mi    = static_cast<int>(M);
    const auto shift = [&](int i) { return (i < Mi / 2 ? i : i - Mi); };

    std::vector<Peak> perYaw;
    Peak              best;
    CMatrixFloat      bestCorr;
    double            bestLocalNorm = 1;

    const int    nYaws = std::max(1, static_cast<int>(std::round(360.0 / params.yaw_step_deg)));
    CMatrixFloat lMat;
    CMatrixFloat lRe;
    CMatrixFloat lIm;
    for (int iy = 0; iy < nYaws; iy++)
    {
        const double yaw = mrpt::DEG2RAD(360.0 * iy / nYaws);
        Grid         lGrid(N, res);
        lGrid.insert(pl, std::cos(yaw), std::sin(yaw), 0, 0, 0);
        const double lNorm = lGrid.to_matrix(params.max_cell_height, lMat, M);
        if (lNorm <= 0)
        {
            return result;
        }
        mrpt::math::dft2_real(lMat, lRe, lIm);

        // Cross-correlation: G * conj(L)
        pRe.asEigen() = gRe.asEigen().array() * lRe.asEigen().array() +
                        gIm.asEigen().array() * lIm.asEigen().array();
        pIm.asEigen() = gIm.asEigen().array() * lRe.asEigen().array() -
                        gRe.asEigen().array() * lIm.asEigen().array();
        mrpt::math::idft2_real(pRe, pIm, corr);

        Eigen::Index bi   = 0;
        Eigen::Index bj   = 0;
        const double peak = static_cast<double>(corr.asEigen().maxCoeff(&bi, &bj));
        Peak         p;
        p.score = peak * invScale / (gNorm * lNorm);
        p.yaw   = yaw;
        // corr(d) = sum_i g(i + d) l(i): local cells move by +d into the global grid
        p.dx = shift(static_cast<int>(bi)) * res;
        p.dy = shift(static_cast<int>(bj)) * res;
        perYaw.push_back(p);
        if (p.score > best.score)
        {
            best          = p;
            bestCorr      = corr;
            bestLocalNorm = lNorm;
        }
    }

    // Ambiguity: the best clearly different peak, at another yaw or elsewhere in
    // the best yaw correlation (e.g. repetitive structures along a direction).
    const double minYaw  = mrpt::DEG2RAD(params.ambiguity_min_yaw_deg);
    const double minDist = params.ambiguity_min_distance;
    double       second  = 0;
    for (const auto& p : perYaw)
    {
        const double dYaw = std::abs(mrpt::math::wrapToPi(p.yaw - best.yaw));
        if (dYaw > minYaw || std::hypot(p.dx - best.dx, p.dy - best.dy) > minDist)
        {
            second = std::max(second, p.score);
        }
    }
    for (int i = 0; i < Mi; i++)
    {
        for (int j = 0; j < Mi; j++)
        {
            if (std::hypot(shift(i) * res - best.dx, shift(j) * res - best.dy) <= minDist)
            {
                continue;
            }
            const double s = bestCorr(i, j) * invScale / (gNorm * bestLocalNorm);
            second         = std::max(second, s);
        }
    }
    result.score           = best.score;
    result.ambiguity_ratio = best.score / std::max(second, 1e-9);

    // z: median difference of the lowest point (ground) of the cells seen by both
    Grid lBest(N, res);
    lBest.insert(pl, std::cos(best.yaw), std::sin(best.yaw), best.dx, best.dy, 0);
    std::vector<double> dz;
    for (size_t k = 0; k < N * N; k++)
    {
        if (gGrid.occupied(k) && lBest.occupied(k))
        {
            dz.push_back(static_cast<double>(gGrid.zmin[k]) - lBest.zmin[k]);
        }
    }
    double           z               = 0;
    constexpr size_t MIN_CELLS_FOR_Z = 20;
    if (dz.size() >= MIN_CELLS_FOR_Z)
    {
        std::nth_element(dz.begin(), dz.begin() + static_cast<long>(dz.size() / 2), dz.end());
        z = dz[dz.size() / 2];
    }

    // Back from the leveled frames: global <- leveled global <- leveled local <- local
    const auto gLev =
        mrpt::poses::CPose3D::FromYawPitchRoll(0, globalAttitude.pitch(), globalAttitude.roll());
    const auto lLev =
        mrpt::poses::CPose3D::FromYawPitchRoll(0, localAttitude.pitch(), localAttitude.roll());
    const auto inLeveled =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(best.dx, best.dy, z, best.yaw, 0, 0);
    result.pose = (-gLev) + inLeveled + lLev;

    result.valid =
        result.score >= params.min_score && result.ambiguity_ratio >= params.min_ambiguity_ratio;
    return result;
}

}  // namespace mola::global_registration
