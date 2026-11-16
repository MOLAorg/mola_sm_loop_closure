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
#include <optional>
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
    YAML_LOAD_OPT(coarse_resolution_factor, unsigned int);
    YAML_LOAD_OPT(coarse_yaw_step_deg, double);
    YAML_LOAD_OPT(coarse_candidates, size_t);
    validate();
}

void BevCorrelationParameters::validate() const
{
    ASSERTMSG_(grid_resolution > 0, "bev_correlation: grid_resolution must be > 0");
    ASSERTMSG_(max_range > 0, "bev_correlation: max_range must be > 0");
    ASSERTMSG_(yaw_step_deg > 0, "bev_correlation: yaw_step_deg must be > 0");
    ASSERTMSG_(coarse_yaw_step_deg > 0, "bev_correlation: coarse_yaw_step_deg must be > 0");
    ASSERTMSG_(
        coarse_resolution_factor >= 1, "bev_correlation: coarse_resolution_factor must be >= 1");
    ASSERTMSG_(ambiguity_min_distance > 0, "bev_correlation: ambiguity_min_distance must be > 0");
    ASSERTMSG_(
        2 * max_range / grid_resolution <= static_cast<double>(MAX_GRID_CELLS),
        mrpt::format(
            "bev_correlation: grids of 2*max_range/grid_resolution = %.0f cells per side, more "
            "than %zu: raise grid_resolution or lower max_range",
            2 * max_range / grid_resolution, MAX_GRID_CELLS));
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

/** Leveled points within maxRange (horizontal) of the origin. */
Points level(
    const mrpt::maps::CPointsMap& pc, const mrpt::poses::CPose3D& attitude, double maxRange)
{
    const auto R = mrpt::poses::CPose3D::FromYawPitchRoll(0, attitude.pitch(), attitude.roll())
                       .getRotationMatrix();
    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();
    Points      out;
    out.x.reserve(xs.size());
    out.y.reserve(xs.size());
    out.z.reserve(xs.size());
    const double maxRange2 = maxRange * maxRange;
    for (size_t i = 0; i < xs.size(); i++)
    {
        const double x = R(0, 0) * xs[i] + R(0, 1) * ys[i] + R(0, 2) * zs[i];
        const double y = R(1, 0) * xs[i] + R(1, 1) * ys[i] + R(1, 2) * zs[i];
        const double z = R(2, 0) * xs[i] + R(2, 1) * ys[i] + R(2, 2) * zs[i];
        // Also drops non-finite points:
        if (!(x * x + y * y <= maxRange2) || !std::isfinite(z))
        {
            continue;
        }
        out.x.push_back(static_cast<float>(x));
        out.y.push_back(static_cast<float>(y));
        out.z.push_back(static_cast<float>(z));
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
            const double fx = (x + half) / res;
            const double fy = (y + half) / res;
            const auto   n  = static_cast<double>(N);
            // Bounds checked before the integer conversion (also rejects NaN):
            if (!(fx >= 0 && fx < n && fy >= 0 && fy < n))
            {
                continue;
            }
            const size_t k = static_cast<size_t>(fx) * N + static_cast<size_t>(fy);
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

/** Correlates rotated versions of the local cloud against the global grid at
 *  one resolution. */
class Correlator
{
   public:
    Correlator(const Points& global, double res, double maxRange, double maxHeight)
        : res_(res),
          maxHeight_(maxHeight),
          N_(next_pow2(static_cast<size_t>(std::ceil(2 * maxRange / res)))),
          M_(2 * N_),  // zero padding: linear, not circular, correlation
          grid_(N_, res)
    {
        grid_.insert(global, 1, 0, 0, 0, 0);
        CMatrixFloat gMat;
        norm_ = grid_.to_matrix(maxHeight_, gMat, M_);
        if (norm_ <= 0)
        {
            return;
        }
        mrpt::math::dft2_real(gMat, re_, im_);

        // Scale of the inverse transform, from the autocorrelation at zero shift:
        CMatrixFloat pRe(static_cast<int>(M_), static_cast<int>(M_));
        CMatrixFloat pIm(static_cast<int>(M_), static_cast<int>(M_));
        pRe.asEigen() = re_.asEigen().array().square() + im_.asEigen().array().square();
        pIm.setZero();
        CMatrixFloat corr;
        mrpt::math::idft2_real(pRe, pIm, corr);
        invScale_ = (norm_ * norm_) / static_cast<double>(corr(0, 0));
    }

    [[nodiscard]] bool        ok() const { return norm_ > 0; }
    [[nodiscard]] const Grid& grid() const { return grid_; }
    [[nodiscard]] double      res() const { return res_; }

    /** Best translation for the local points rotated by yaw. If secondPeak is
     *  given, it receives the best score at least minDist away from it. */
    Peak eval(const Points& local, double yaw, double minDist = 0, double* secondPeak = nullptr)
    {
        Peak p;
        if (!ok())
        {
            return p;
        }
        Grid lGrid(N_, res_);
        lGrid.insert(local, std::cos(yaw), std::sin(yaw), 0, 0, 0);
        const double lNorm = lGrid.to_matrix(maxHeight_, lMat_, M_);
        if (lNorm <= 0)
        {
            return p;
        }
        mrpt::math::dft2_real(lMat_, lRe_, lIm_);
        // Cross-correlation: G * conj(L)
        pRe_.resize(re_.rows(), re_.cols());
        pIm_.resize(re_.rows(), re_.cols());
        pRe_.asEigen() = re_.asEigen().array() * lRe_.asEigen().array() +
                         im_.asEigen().array() * lIm_.asEigen().array();
        pIm_.asEigen() = im_.asEigen().array() * lRe_.asEigen().array() -
                         re_.asEigen().array() * lIm_.asEigen().array();
        mrpt::math::idft2_real(pRe_, pIm_, corr_);

        Eigen::Index bi    = 0;
        Eigen::Index bj    = 0;
        const double peak  = static_cast<double>(corr_.asEigen().maxCoeff(&bi, &bj));
        const double scale = invScale_ / (norm_ * lNorm);
        p.score            = peak * scale;
        p.yaw              = yaw;
        // corr(d) = sum_i g(i + d) l(i): local cells move by +d into the global grid
        p.dx = shift(static_cast<int>(bi)) * res_;
        p.dy = shift(static_cast<int>(bj)) * res_;

        if (secondPeak)
        {
            const auto Mi = static_cast<int>(M_);
            double     s2 = 0;
            for (int i = 0; i < Mi; i++)
            {
                for (int j = 0; j < Mi; j++)
                {
                    if (std::hypot(shift(i) * res_ - p.dx, shift(j) * res_ - p.dy) > minDist)
                    {
                        s2 = std::max(s2, static_cast<double>(corr_(i, j)) * scale);
                    }
                }
            }
            *secondPeak = s2;
        }
        return p;
    }

   private:
    double       res_;
    double       maxHeight_;
    size_t       N_;
    size_t       M_;
    Grid         grid_;
    double       norm_     = 0;
    double       invScale_ = 1;
    CMatrixFloat re_;
    CMatrixFloat im_;
    CMatrixFloat lMat_;
    CMatrixFloat lRe_;
    CMatrixFloat lIm_;
    CMatrixFloat pRe_;
    CMatrixFloat pIm_;
    CMatrixFloat corr_;

    [[nodiscard]] int shift(int i) const
    {
        const auto Mi = static_cast<int>(M_);
        return i < Mi / 2 ? i : i - Mi;
    }
};

bool distinct(const Peak& a, const Peak& b, double minYaw, double minDist)
{
    return std::abs(mrpt::math::wrapToPi(a.yaw - b.yaw)) > minYaw ||
           std::hypot(a.dx - b.dx, a.dy - b.dy) > minDist;
}

}  // namespace

BevCorrelationResult bev_correlation(
    const mrpt::maps::CPointsMap& global, const mrpt::poses::CPose3D& globalAttitude,
    const mrpt::maps::CPointsMap& local, const mrpt::poses::CPose3D& localAttitude,
    const BevCorrelationParameters& params)
{
    BevCorrelationResult result;
    params.validate();

    const Points pg = level(global, globalAttitude, params.max_range);
    const Points pl = level(local, localAttitude, params.max_range);

    const double minYaw = mrpt::DEG2RAD(params.ambiguity_min_yaw_deg);
    // Alternatives closer than two cells are the same peak:
    const double minDist = std::max(params.ambiguity_min_distance, 2 * params.grid_resolution);

    // Stage 1, coarse: every yaw on a coarser grid.
    const bool   twoStages = params.coarse_resolution_factor > 1;
    const double coarseRes =
        params.grid_resolution * static_cast<double>(params.coarse_resolution_factor);
    const double coarseStep = twoStages ? params.coarse_yaw_step_deg : params.yaw_step_deg;
    Correlator   coarse(
          pg, twoStages ? coarseRes : params.grid_resolution, params.max_range,
          params.max_cell_height);
    if (!coarse.ok())
    {
        return result;
    }
    std::vector<Peak> coarsePeaks;
    const int         nYaws = std::max(1, static_cast<int>(std::round(360.0 / coarseStep)));
    for (int iy = 0; iy < nYaws; iy++)
    {
        coarsePeaks.push_back(coarse.eval(pl, mrpt::DEG2RAD(360.0 * iy / nYaws)));
    }
    std::sort(
        coarsePeaks.begin(), coarsePeaks.end(),
        [](const Peak& a, const Peak& b) { return a.score > b.score; });

    // The best few distinct coarse candidates; at least two, so that the
    // ambiguity ratio always sees a clearly different alternative:
    std::vector<Peak> candidates;
    for (const auto& p : coarsePeaks)
    {
        if (candidates.size() >= std::max<size_t>(2, params.coarse_candidates))
        {
            break;
        }
        const bool isNew = std::all_of(
            candidates.begin(), candidates.end(),
            [&](const Peak& c) { return distinct(p, c, minYaw, minDist); });
        if (isNew)
        {
            candidates.push_back(p);
        }
    }

    // Stage 2, fine: yaws around each candidate, at the full resolution.
    std::optional<Correlator> fineOwned;
    if (twoStages)
    {
        fineOwned.emplace(pg, params.grid_resolution, params.max_range, params.max_cell_height);
        if (!fineOwned->ok())
        {
            return result;
        }
    }
    Correlator& fine = twoStages ? *fineOwned : coarse;

    // Each candidate refined in yaw (two stages), with the best score of its
    // correlation at least minDist away from its peak:
    std::vector<Peak>   finePeaks;
    std::vector<double> sameYawSeconds;
    for (const auto& c : candidates)
    {
        Peak   bestHere;
        double secondHere = 0;
        if (!twoStages)
        {
            bestHere = fine.eval(pl, c.yaw, minDist, &secondHere);
        }
        else
        {
            const int half =
                static_cast<int>(std::ceil(0.5 * coarseStep / params.yaw_step_deg)) + 1;
            for (int k = -half; k <= half; k++)
            {
                double     s2 = 0;
                const auto p =
                    fine.eval(pl, c.yaw + mrpt::DEG2RAD(k * params.yaw_step_deg), minDist, &s2);
                if (p.score > bestHere.score)
                {
                    bestHere   = p;
                    secondHere = s2;
                }
            }
        }
        finePeaks.push_back(bestHere);
        sameYawSeconds.push_back(secondHere);
    }
    const auto bestIdx = static_cast<size_t>(std::distance(
        finePeaks.begin(), std::max_element(
                               finePeaks.begin(), finePeaks.end(),
                               [](const Peak& a, const Peak& b) { return a.score < b.score; })));
    const Peak best    = finePeaks.at(bestIdx);

    // Ambiguity: the best clearly different peak, among the other candidates and
    // elsewhere in the best yaw correlation (repetitive structures along a direction).
    double second = 0;
    for (const auto& p : finePeaks)
    {
        if (distinct(p, best, minYaw, minDist))
        {
            second = std::max(second, p.score);
        }
    }
    second = std::max(second, sameYawSeconds.at(bestIdx));

    result.score           = best.score;
    result.ambiguity_ratio = best.score > 0 ? best.score / std::max(second, 1e-9) : 0.0;

    // z: median difference of the lowest point (ground) of the cells seen by both
    const Grid& gGrid = fine.grid();
    Grid        lBest(gGrid.N, gGrid.res);
    lBest.insert(pl, std::cos(best.yaw), std::sin(best.yaw), best.dx, best.dy, 0);
    std::vector<double> dz;
    for (size_t k = 0; k < gGrid.N * gGrid.N; k++)
    {
        if (gGrid.occupied(k) && lBest.occupied(k))
        {
            dz.push_back(static_cast<double>(gGrid.zmin[k]) - lBest.zmin[k]);
        }
    }
    // Too little overlap to estimate z: not a usable result.
    constexpr size_t MIN_CELLS_FOR_Z = 20;
    if (dz.size() < MIN_CELLS_FOR_Z)
    {
        return result;
    }
    std::nth_element(dz.begin(), dz.begin() + static_cast<long>(dz.size() / 2), dz.end());
    const double z = dz[dz.size() / 2];

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
