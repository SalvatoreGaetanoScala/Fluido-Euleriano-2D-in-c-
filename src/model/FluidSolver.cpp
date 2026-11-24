#include "model/FluidSolver.hpp"

#include <algorithm>
#include <cmath>

namespace fluid::model
{

    FluidSolver::FluidSolver(const GridConfig &gridConfig, const SolverParams &params)
        : gridConfig_(gridConfig),
          params_(params),
          grid_(gridConfig),
          curl_(grid_.size(), 0.0f),
          omega_(grid_.size(), 0.0f) {}

    void FluidSolver::reset()
    {
        grid_.resetAll();
        std::fill(curl_.begin(), curl_.end(), 0.0f);
        std::fill(omega_.begin(), omega_.end(), 0.0f);
    }

    void FluidSolver::addDensityBrush(int cx, int cy, int radius, float amount)
    {
        const int r = std::max(1, radius);
        const int minX = std::max(1, cx - r);
        const int maxX = std::min(grid_.width(), cx + r);
        const int minY = std::max(1, cy - r);
        const int maxY = std::min(grid_.height(), cy + r);
        const float r2 = static_cast<float>(r * r);

        for (int y = minY; y <= maxY; ++y)
        {
            const float dy = static_cast<float>(y - cy);
            for (int x = minX; x <= maxX; ++x)
            {
                const float dx = static_cast<float>(x - cx);
                const float d2 = dx * dx + dy * dy;
                if (d2 <= r2)
                {
                    const std::size_t idx = grid_.index(x, y);
                    if (!grid_.isSolidAt(idx))
                    {
                        const float falloff = 1.0f - (d2 / r2);
                        grid_.density[idx] = std::clamp(
                            grid_.density[idx] + amount * falloff, 0.0f, 10.0f);
                    }
                }
            }
        }
    }

    void FluidSolver::addVelocityBrush(int cx, int cy, int radius, float forceX, float forceY)
    {
        const int r = std::max(1, radius);
        const int minX = std::max(1, cx - r);
        const int maxX = std::min(grid_.width(), cx + r);
        const int minY = std::max(1, cy - r);
        const int maxY = std::min(grid_.height(), cy + r);
        const float r2 = static_cast<float>(r * r);

        for (int y = minY; y <= maxY; ++y)
        {
            const float dy = static_cast<float>(y - cy);
            for (int x = minX; x <= maxX; ++x)
            {
                const float dx = static_cast<float>(x - cx);
                const float d2 = dx * dx + dy * dy;
                if (d2 <= r2)
                {
                    const std::size_t idx = grid_.index(x, y);
                    if (!grid_.isSolidAt(idx))
                    {
                        const float falloff = 1.0f - (d2 / r2);
                        grid_.vx[idx] += forceX * falloff;
                        grid_.vy[idx] += forceY * falloff;
                    }
                }
            }
        }
    }

    void FluidSolver::setObstacleBrush(int cx, int cy, int radius, bool solid)
    {
        grid_.setObstacleCircle(cx, cy, radius, solid);
    }

    void FluidSolver::step(float dt)
    {
        if (dt <= 0.0f)
        {
            return;
        }

        applyExternalForces(dt);
        stepVelocity(dt);
        stepDensity(dt);
    }

    void FluidSolver::applyExternalForces(float dt)
    {
        if (continuousSource_)
        {
            const int midX = grid_.width() / 2;
            const int srcY = std::max(2, grid_.height() - 5);
            addDensityBrush(midX, srcY, std::max(3, grid_.width() / 25), 2.5f);
            addVelocityBrush(midX, srcY, std::max(3, grid_.width() / 25), 0.0f, -18.0f);
        }

        const int w = grid_.width();
        const int h = grid_.height();
        for (int y = 1; y <= h; ++y)
        {
            for (int x = 1; x <= w; ++x)
            {
                const std::size_t idx = grid_.index(x, y);
                if (grid_.isSolidAt(idx))
                {
                    continue;
                }
                // Galleggiamento (fumo più leggero sale verso l'alto, y decrescente) + gravità
                const float d = grid_.density[idx];
                grid_.vy[idx] += (-params_.buoyancy * d + params_.gravityY) * dt;
            }
        }
    }

    void FluidSolver::stepVelocity(float dt)
    {
        if (params_.vorticityStrength > 0.0f)
        {
            applyVorticityConfinement(dt);
        }

        std::swap(grid_.prevVx, grid_.vx);
        std::swap(grid_.prevVy, grid_.vy);

        diffuse(grid_.vx, grid_.prevVx, params_.viscosity, dt, BoundaryMode::FlipX);
        diffuse(grid_.vy, grid_.prevVy, params_.viscosity, dt, BoundaryMode::FlipY);

        project(dt);

        std::swap(grid_.prevVx, grid_.vx);
        std::swap(grid_.prevVy, grid_.vy);

        advect(grid_.vx, grid_.prevVx, grid_.prevVx, grid_.prevVy, dt, BoundaryMode::FlipX);
        advect(grid_.vy, grid_.prevVy, grid_.prevVx, grid_.prevVy, dt, BoundaryMode::FlipY);

        project(dt);

        const float damp = std::clamp(1.0f - params_.velocityDamping * dt, 0.0f, 1.0f);
        for (std::size_t i = 0; i < grid_.size(); ++i)
        {
            if (grid_.isSolidAt(i))
            {
                grid_.vx[i] = 0.0f;
                grid_.vy[i] = 0.0f;
            }
            else
            {
                grid_.vx[i] *= damp;
                grid_.vy[i] *= damp;
            }
        }
    }

    void FluidSolver::stepDensity(float dt)
    {
        std::swap(grid_.prevDensity, grid_.density);

        diffuse(grid_.density, grid_.prevDensity, params_.diffusion, dt, BoundaryMode::None);

        std::swap(grid_.prevDensity, grid_.density);
        advect(grid_.density, grid_.prevDensity, grid_.vx, grid_.vy, dt, BoundaryMode::None);

        const float decay = std::clamp(1.0f - params_.densityDissipation * dt, 0.0f, 1.0f);
        for (std::size_t i = 0; i < grid_.size(); ++i)
        {
            if (grid_.isSolidAt(i))
            {
                grid_.density[i] = 0.0f;
            }
            else
            {
                grid_.density[i] = std::max(0.0f, grid_.density[i] * decay);
            }
        }
    }

    void FluidSolver::diffuse(std::vector<float> &field, const std::vector<float> &prevField,
                              float diffRate, float dt, BoundaryMode mode)
    {
        const float a = dt * diffRate * static_cast<float>(grid_.width()) *
                        static_cast<float>(grid_.height());
        if (a <= 0.0f)
        {
            field = prevField;
            enforceBoundary(field, mode);
            return;
        }

        const int w = grid_.width();
        const int h = grid_.height();
        const float invDenom = 1.0f / (1.0f + 4.0f * a);

        for (int iter = 0; iter < params_.solverIterations; ++iter)
        {
            for (int y = 1; y <= h; ++y)
            {
                for (int x = 1; x <= w; ++x)
                {
                    const std::size_t idx = grid_.index(x, y);
                    if (grid_.isSolidAt(idx))
                    {
                        field[idx] = 0.0f;
                        continue;
                    }

                    const float sumNeighbors =
                        field[grid_.index(x - 1, y)] +
                        field[grid_.index(x + 1, y)] +
                        field[grid_.index(x, y - 1)] +
                        field[grid_.index(x, y + 1)];

                    field[idx] = (prevField[idx] + a * sumNeighbors) * invDenom;
                }
            }
            enforceBoundary(field, mode);
        }
    }

    void FluidSolver::advect(std::vector<float> &field, const std::vector<float> &prevField,
                             const std::vector<float> &velX, const std::vector<float> &velY,
                             float dt, BoundaryMode mode)
    {
        const int w = grid_.width();
        const int h = grid_.height();
        const float fw = static_cast<float>(w);
        const float fh = static_cast<float>(h);
        const float dt0x = dt * fw;
        const float dt0y = dt * fh;

        for (int y = 1; y <= h; ++y)
        {
            for (int x = 1; x <= w; ++x)
            {
                const std::size_t idx = grid_.index(x, y);
                if (grid_.isSolidAt(idx))
                {
                    field[idx] = 0.0f;
                    continue;
                }

                float backX = static_cast<float>(x) - dt0x * velX[idx];
                float backY = static_cast<float>(y) - dt0y * velY[idx];

                backX = std::clamp(backX, 0.5f, fw + 0.5f);
                backY = std::clamp(backY, 0.5f, fh + 0.5f);

                const int i0 = static_cast<int>(std::floor(backX));
                const int i1 = i0 + 1;
                const int j0 = static_cast<int>(std::floor(backY));
                const int j1 = j0 + 1;

                const float s1 = backX - static_cast<float>(i0);
                const float s0 = 1.0f - s1;
                const float t1 = backY - static_cast<float>(j0);
                const float t0 = 1.0f - t1;

                const float p00 = prevField[grid_.index(i0, j0)];
                const float p10 = prevField[grid_.index(i1, j0)];
                const float p01 = prevField[grid_.index(i0, j1)];
                const float p11 = prevField[grid_.index(i1, j1)];

                field[idx] = s0 * (t0 * p00 + t1 * p01) +
                             s1 * (t0 * p10 + t1 * p11);
            }
        }

        enforceBoundary(field, mode);
    }

    void FluidSolver::project(float /*dt*/)
    {
        const int w = grid_.width();
        const int h = grid_.height();
        const float hx = 1.0f / static_cast<float>(w);
        const float hy = 1.0f / static_cast<float>(h);

        std::fill(grid_.divergence.begin(), grid_.divergence.end(), 0.0f);
        std::fill(grid_.pressure.begin(), grid_.pressure.end(), 0.0f);

        for (int y = 1; y <= h; ++y)
        {
            for (int x = 1; x <= w; ++x)
            {
                const std::size_t idx = grid_.index(x, y);
                if (grid_.isSolidAt(idx))
                {
                    continue;
                }

                const float uxL = grid_.isSolid(x - 1, y) ? 0.0f : grid_.vx[grid_.index(x - 1, y)];
                const float uxR = grid_.isSolid(x + 1, y) ? 0.0f : grid_.vx[grid_.index(x + 1, y)];
                const float uyB = grid_.isSolid(x, y - 1) ? 0.0f : grid_.vy[grid_.index(x, y - 1)];
                const float uyT = grid_.isSolid(x, y + 1) ? 0.0f : grid_.vy[grid_.index(x, y + 1)];

                grid_.divergence[idx] = -0.5f * (hx * (uxR - uxL) + hy * (uyT - uyB));
            }
        }

        enforceBoundary(grid_.divergence, BoundaryMode::None);
        enforceBoundary(grid_.pressure, BoundaryMode::None);

        for (int iter = 0; iter < params_.solverIterations; ++iter)
        {
            for (int y = 1; y <= h; ++y)
            {
                for (int x = 1; x <= w; ++x)
                {
                    const std::size_t idx = grid_.index(x, y);
                    if (grid_.isSolidAt(idx))
                    {
                        grid_.pressure[idx] = 0.0f;
                        continue;
                    }

                    const float pL = grid_.isSolid(x - 1, y) ? grid_.pressure[idx] : grid_.pressure[grid_.index(x - 1, y)];
                    const float pR = grid_.isSolid(x + 1, y) ? grid_.pressure[idx] : grid_.pressure[grid_.index(x + 1, y)];
                    const float pB = grid_.isSolid(x, y - 1) ? grid_.pressure[idx] : grid_.pressure[grid_.index(x, y - 1)];
                    const float pT = grid_.isSolid(x, y + 1) ? grid_.pressure[idx] : grid_.pressure[grid_.index(x, y + 1)];

                    grid_.pressure[idx] = 0.25f * (grid_.divergence[idx] + pL + pR + pB + pT);
                }
            }
            enforceBoundary(grid_.pressure, BoundaryMode::None);
        }

        const float scaleX = 0.5f / hx;
        const float scaleY = 0.5f / hy;
        for (int y = 1; y <= h; ++y)
        {
            for (int x = 1; x <= w; ++x)
            {
                const std::size_t idx = grid_.index(x, y);
                if (grid_.isSolidAt(idx))
                {
                    grid_.vx[idx] = 0.0f;
                    grid_.vy[idx] = 0.0f;
                    continue;
                }

                const float pL = grid_.isSolid(x - 1, y) ? grid_.pressure[idx] : grid_.pressure[grid_.index(x - 1, y)];
                const float pR = grid_.isSolid(x + 1, y) ? grid_.pressure[idx] : grid_.pressure[grid_.index(x + 1, y)];
                const float pB = grid_.isSolid(x, y - 1) ? grid_.pressure[idx] : grid_.pressure[grid_.index(x, y - 1)];
                const float pT = grid_.isSolid(x, y + 1) ? grid_.pressure[idx] : grid_.pressure[grid_.index(x, y + 1)];

                grid_.vx[idx] -= scaleX * (pR - pL);
                grid_.vy[idx] -= scaleY * (pT - pB);
            }
        }

        enforceBoundary(grid_.vx, BoundaryMode::FlipX);
        enforceBoundary(grid_.vy, BoundaryMode::FlipY);
    }

    void FluidSolver::applyVorticityConfinement(float dt)
    {
        const int w = grid_.width();
        const int h = grid_.height();
        std::fill(omega_.begin(), omega_.end(), 0.0f);

        for (int y = 1; y <= h; ++y)
        {
            for (int x = 1; x <= w; ++x)
            {
                const float dvyDx = 0.5f * (grid_.vy[grid_.index(x + 1, y)] - grid_.vy[grid_.index(x - 1, y)]);
                const float dvxDy = 0.5f * (grid_.vx[grid_.index(x, y + 1)] - grid_.vx[grid_.index(x, y - 1)]);
                const float curlVal = dvyDx - dvxDy;
                omega_[grid_.index(x, y)] = curlVal;
                curl_[grid_.index(x, y)] = std::abs(curlVal);
            }
        }

        constexpr float eps = 1e-6f;
        for (int y = 2; y < h; ++y)
        {
            for (int x = 2; x < w; ++x)
            {
                const std::size_t idx = grid_.index(x, y);
                if (grid_.isSolidAt(idx))
                {
                    continue;
                }

                const float absC = std::abs(omega_[idx]);
                const float dwDx = 0.5f * (std::abs(omega_[grid_.index(x + 1, y)]) -
                                           std::abs(omega_[grid_.index(x - 1, y)]));
                const float dwDy = 0.5f * (std::abs(omega_[grid_.index(x, y + 1)]) -
                                           std::abs(omega_[grid_.index(x, y - 1)]));

                const float len = std::sqrt(dwDx * dwDx + dwDy * dwDy) + eps;
                const float nx = dwDx / len;
                const float ny = dwDy / len;

                const float fx = ny * omega_[idx] * params_.vorticityStrength;
                const float fy = -nx * omega_[idx] * params_.vorticityStrength;

                grid_.vx[idx] += fx * dt * (1.0f + absC * 0.1f);
                grid_.vy[idx] += fy * dt * (1.0f + absC * 0.1f);
            }
        }
    }

    void FluidSolver::enforceBoundary(std::vector<float> &field, BoundaryMode mode)
    {
        const int w = grid_.width();
        const int h = grid_.height();

        const float signX = (mode == BoundaryMode::FlipX) ? -1.0f : 1.0f;
        const float signY = (mode == BoundaryMode::FlipY) ? -1.0f : 1.0f;

        for (int y = 1; y <= h; ++y)
        {
            field[grid_.index(0, y)] = signX * field[grid_.index(1, y)];
            field[grid_.index(w + 1, y)] = signX * field[grid_.index(w, y)];
        }

        for (int x = 1; x <= w; ++x)
        {
            field[grid_.index(x, 0)] = signY * field[grid_.index(x, 1)];
            field[grid_.index(x, h + 1)] = signY * field[grid_.index(x, h)];
        }

        field[grid_.index(0, 0)] = 0.5f * (field[grid_.index(1, 0)] + field[grid_.index(0, 1)]);
        field[grid_.index(w + 1, 0)] = 0.5f * (field[grid_.index(w, 0)] + field[grid_.index(w + 1, 1)]);
        field[grid_.index(0, h + 1)] = 0.5f * (field[grid_.index(1, h + 1)] + field[grid_.index(0, h)]);
        field[grid_.index(w + 1, h + 1)] = 0.5f * (field[grid_.index(w, h + 1)] + field[grid_.index(w + 1, h)]);
    }

    void FluidSolver::computeCurlMagnitude(std::vector<float> &outCurl) const
    {
        outCurl.resize(grid_.size(), 0.0f);
        const int w = grid_.width();
        const int h = grid_.height();

        for (int y = 1; y <= h; ++y)
        {
            for (int x = 1; x <= w; ++x)
            {
                const float dvyDx = 0.5f * (grid_.vy[grid_.index(x + 1, y)] - grid_.vy[grid_.index(x - 1, y)]);
                const float dvxDy = 0.5f * (grid_.vx[grid_.index(x, y + 1)] - grid_.vx[grid_.index(x, y - 1)]);
                outCurl[grid_.index(x, y)] = std::abs(dvyDx - dvxDy);
            }
        }
    }

} // namespace fluid::model
