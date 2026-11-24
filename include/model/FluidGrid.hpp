#pragma once

#include <cstdint>
#include <vector>

namespace fluid::model
{

    struct GridConfig
    {
        int width{160};       // Celle interne orizzontali (esclusi bordi)
        int height{120};      // Celle interne verticali (esclusi bordi)
        float cellSize{1.0f}; // Dimensione spaziale di una cella
    };

    // Griglia Euleriana 2D con bordo fantasma (width + 2) x (height + 2)
    // e indicizzazione 1D contigua per migliore cache locality.
    class FluidGrid
    {
    public:
        explicit FluidGrid(const GridConfig &config);

        [[nodiscard]] int width() const noexcept { return width_; }
        [[nodiscard]] int height() const noexcept { return height_; }
        [[nodiscard]] int totalWidth() const noexcept { return width_ + 2; }
        [[nodiscard]] int totalHeight() const noexcept { return height_ + 2; }
        [[nodiscard]] std::size_t size() const noexcept { return density.size(); }
        [[nodiscard]] float cellSize() const noexcept { return cellSize_; }

        // Indice includendo il bordo: x in [0, width + 1], y in [0, height + 1]
        [[nodiscard]] std::size_t index(int x, int y) const noexcept
        {
            return static_cast<std::size_t>(y) * static_cast<std::size_t>(totalWidth()) +
                   static_cast<std::size_t>(x);
        }

        void clearFields();
        void clearObstacles();
        void resetAll();

        void setObstacleCircle(int cx, int cy, int radius, bool solid);
        void setObstacleRect(int x0, int y0, int x1, int y1, bool solid);

        [[nodiscard]] bool isSolid(int x, int y) const noexcept;
        [[nodiscard]] bool isSolidAt(std::size_t idx) const noexcept { return obstacles_[idx] != 0; }

        std::vector<float> density;
        std::vector<float> prevDensity;

        std::vector<float> vx;
        std::vector<float> vy;
        std::vector<float> prevVx;
        std::vector<float> prevVy;

        std::vector<float> pressure;
        std::vector<float> divergence;

    private:
        int width_{0};
        int height_{0};
        float cellSize_{1.0f};
        std::vector<std::uint8_t> obstacles_;
    };

} // namespace fluid::model
