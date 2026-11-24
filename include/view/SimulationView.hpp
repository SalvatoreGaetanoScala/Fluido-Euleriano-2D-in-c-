#pragma once

#include "model/FluidSolver.hpp"
#include "view/FluidTexture.hpp"
#include "raylib.h"

#include <vector>

namespace fluid::view
{

    enum class ViewMode
    {
        SmokeDensity,
        VelocityField,
        Vorticity,
        Pressure
    };

    struct HudState
    {
        bool paused{false};
        bool continuousSource{true};
        bool showVectorArrows{false};
        bool showHelp{true};
        int brushRadius{6};
        ViewMode viewMode{ViewMode::SmokeDensity};
        int substeps{1};
    };

    class SimulationView
    {
    public:
        SimulationView(int screenWidth, int screenHeight, const model::FluidGrid &grid);

        [[nodiscard]] int screenWidth() const noexcept { return screenWidth_; }
        [[nodiscard]] int screenHeight() const noexcept { return screenHeight_; }

        // Converte coordinate schermo -> coordinate griglia interna [1..width], [1..height]
        [[nodiscard]] bool screenToGrid(Vector2 screenPos, const model::FluidGrid &grid,
                                        int &outGx, int &outGy) const noexcept;

        void render(const model::FluidSolver &solver, const HudState &hud);

    private:
        void updatePixelBuffer(const model::FluidSolver &solver, ViewMode mode);
        void drawVelocityVectors(const model::FluidGrid &grid) const;
        void drawBrushPreview(const model::FluidGrid &grid, int brushRadius) const;
        void drawHud(const HudState &hud) const;

        int screenWidth_{1280};
        int screenHeight_{800};
        FluidTexture fluidTexture_;
        std::vector<float> curlBuffer_;
    };

} // namespace fluid::view
