#include "model/FluidGrid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fluid::model
{

    FluidGrid::FluidGrid(const GridConfig &config)
        : width_(std::max(8, config.width)),
          height_(std::max(8, config.height)),
          cellSize_(std::max(0.001f, config.cellSize))
    {
        const std::size_t n = static_cast<std::size_t>(totalWidth()) *
                              static_cast<std::size_t>(totalHeight());

        density.assign(n, 0.0f);
        prevDensity.assign(n, 0.0f);
        vx.assign(n, 0.0f);
        vy.assign(n, 0.0f);
        prevVx.assign(n, 0.0f);
        prevVy.assign(n, 0.0f);
        pressure.assign(n, 0.0f);
        divergence.assign(n, 0.0f);
        obstacles_.assign(n, 0);
    }

    void FluidGrid::clearFields()
    {
        std::fill(density.begin(), density.end(), 0.0f);
        std::fill(prevDensity.begin(), prevDensity.end(), 0.0f);
        std::fill(vx.begin(), vx.end(), 0.0f);
        std::fill(vy.begin(), vy.end(), 0.0f);
        std::fill(prevVx.begin(), prevVx.end(), 0.0f);
        std::fill(prevVy.begin(), prevVy.end(), 0.0f);
        std::fill(pressure.begin(), pressure.end(), 0.0f);
        std::fill(divergence.begin(), divergence.end(), 0.0f);
    }

    void FluidGrid::clearObstacles()
    {
        std::fill(obstacles_.begin(), obstacles_.end(), 0);
    }

    void FluidGrid::resetAll()
    {
        clearFields();
        clearObstacles();
    }

    void FluidGrid::setObstacleCircle(int cx, int cy, int radius, bool solid)
    {
        const int r = std::max(0, radius);
        const int minX = std::max(1, cx - r);
        const int maxX = std::min(width_, cx + r);
        const int minY = std::max(1, cy - r);
        const int maxY = std::min(height_, cy + r);
        const int r2 = r * r;

        for (int y = minY; y <= maxY; ++y)
        {
            const int dy = y - cy;
            for (int x = minX; x <= maxX; ++x)
            {
                const int dx = x - cx;
                if (dx * dx + dy * dy <= r2)
                {
                    const std::size_t idx = index(x, y);
                    obstacles_[idx] = solid ? 1 : 0;
                    if (solid)
                    {
                        vx[idx] = 0.0f;
                        vy[idx] = 0.0f;
                        density[idx] = 0.0f;
                    }
                }
            }
        }
    }

    void FluidGrid::setObstacleRect(int x0, int y0, int x1, int y1, bool solid)
    {
        const int minX = std::clamp(std::min(x0, x1), 1, width_);
        const int maxX = std::clamp(std::max(x0, x1), 1, width_);
        const int minY = std::clamp(std::min(y0, y1), 1, height_);
        const int maxY = std::clamp(std::max(y0, y1), 1, height_);

        for (int y = minY; y <= maxY; ++y)
        {
            for (int x = minX; x <= maxX; ++x)
            {
                const std::size_t idx = index(x, y);
                obstacles_[idx] = solid ? 1 : 0;
                if (solid)
                {
                    vx[idx] = 0.0f;
                    vy[idx] = 0.0f;
                    density[idx] = 0.0f;
                }
            }
        }
    }

    bool FluidGrid::isSolid(int x, int y) const noexcept
    {
        if (x < 0 || x > width_ + 1 || y < 0 || y > height_ + 1)
        {
            return true;
        }
        return obstacles_[index(x, y)] != 0;
    }

} // namespace fluid::model
