#include "view/SimulationView.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fluid::view
{

    namespace
    {

        Color mapSmokeColor(float density) noexcept
        {
            const float d = std::clamp(density, 0.0f, 1.5f) / 1.5f;
            const auto r = static_cast<unsigned char>(std::clamp(18.0f + d * 215.0f, 0.0f, 255.0f));
            const auto g = static_cast<unsigned char>(std::clamp(22.0f + d * 220.0f, 0.0f, 255.0f));
            const auto b = static_cast<unsigned char>(std::clamp(34.0f + d * 221.0f, 0.0f, 255.0f));
            return Color{r, g, b, 255};
        }

        Color mapVelocityColor(float vx, float vy) noexcept
        {
            const float mag = std::clamp(std::sqrt(vx * vx + vy * vy) * 0.08f, 0.0f, 1.0f);
            const float angle = (std::atan2(vy, vx) + PI) / (2.0f * PI); // [0..1]
            const float h = angle * 360.0f;
            return ColorFromHSV(h, 0.85f, std::clamp(0.12f + mag * 0.88f, 0.0f, 1.0f));
        }

        Color mapScalarHeatmap(float value, float maxVal) noexcept
        {
            const float t = std::clamp(value / std::max(0.0001f, maxVal), 0.0f, 1.0f);
            const auto r = static_cast<unsigned char>(std::clamp(t * 255.0f, 0.0f, 255.0f));
            const auto g = static_cast<unsigned char>(std::clamp((1.0f - std::abs(t - 0.5f) * 2.0f) * 180.0f, 0.0f, 255.0f));
            const auto b = static_cast<unsigned char>(std::clamp((1.0f - t) * 230.0f, 0.0f, 255.0f));
            return Color{r, g, b, 255};
        }

        const char *viewModeName(ViewMode mode) noexcept
        {
            switch (mode)
            {
            case ViewMode::SmokeDensity:
                return "Densita Fumo";
            case ViewMode::VelocityField:
                return "Campo Velocita";
            case ViewMode::Vorticity:
                return "Vorticita";
            case ViewMode::Pressure:
                return "Pressione";
            }
            return "Sconosciuto";
        }

    } // namespace

    SimulationView::SimulationView(int screenWidth, int screenHeight, const model::FluidGrid &grid)
        : screenWidth_(screenWidth),
          screenHeight_(screenHeight),
          fluidTexture_(grid.width(), grid.height()),
          curlBuffer_(grid.size(), 0.0f) {}

    bool SimulationView::screenToGrid(Vector2 screenPos, const model::FluidGrid &grid,
                                      int &outGx, int &outGy) const noexcept
    {
        if (screenPos.x < 0.0f || screenPos.y < 0.0f ||
            screenPos.x >= static_cast<float>(screenWidth_) ||
            screenPos.y >= static_cast<float>(screenHeight_))
        {
            return false;
        }

        const float normX = screenPos.x / static_cast<float>(screenWidth_);
        const float normY = screenPos.y / static_cast<float>(screenHeight_);

        outGx = std::clamp(static_cast<int>(normX * static_cast<float>(grid.width())) + 1,
                           1, grid.width());
        outGy = std::clamp(static_cast<int>(normY * static_cast<float>(grid.height())) + 1,
                           1, grid.height());
        return true;
    }

    void SimulationView::render(const model::FluidSolver &solver, const HudState &hud)
    {
        updatePixelBuffer(solver, hud.viewMode);
        fluidTexture_.upload();

        BeginDrawing();
        ClearBackground(BLACK);

        const Rectangle src{
            0.0f,
            0.0f,
            static_cast<float>(fluidTexture_.width()),
            static_cast<float>(fluidTexture_.height())};
        const Rectangle dst{
            0.0f,
            0.0f,
            static_cast<float>(screenWidth_),
            static_cast<float>(screenHeight_)};

        DrawTexturePro(fluidTexture_.texture(), src, dst, Vector2{0.0f, 0.0f}, 0.0f, WHITE);

        if (hud.showVectorArrows)
        {
            drawVelocityVectors(solver.grid());
        }

        drawBrushPreview(solver.grid(), hud.brushRadius);
        drawHud(hud);

        EndDrawing();
    }

    void SimulationView::updatePixelBuffer(const model::FluidSolver &solver, ViewMode mode)
    {
        const auto &grid = solver.grid();
        auto &pixels = fluidTexture_.pixels();
        const int w = grid.width();
        const int h = grid.height();

        if (mode == ViewMode::Vorticity)
        {
            solver.computeCurlMagnitude(curlBuffer_);
        }

        for (int y = 0; y < h; ++y)
        {
            const int gy = y + 1;
            for (int x = 0; x < w; ++x)
            {
                const int gx = x + 1;
                const std::size_t gIdx = grid.index(gx, gy);
                const std::size_t pIdx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                         static_cast<std::size_t>(x);

                if (grid.isSolidAt(gIdx))
                {
                    pixels[pIdx] = Color{180, 55, 55, 255};
                    continue;
                }

                switch (mode)
                {
                case ViewMode::SmokeDensity:
                    pixels[pIdx] = mapSmokeColor(grid.density[gIdx]);
                    break;
                case ViewMode::VelocityField:
                    pixels[pIdx] = mapVelocityColor(grid.vx[gIdx], grid.vy[gIdx]);
                    break;
                case ViewMode::Vorticity:
                    pixels[pIdx] = mapScalarHeatmap(curlBuffer_[gIdx], 6.0f);
                    break;
                case ViewMode::Pressure:
                    pixels[pIdx] = mapScalarHeatmap(std::abs(grid.pressure[gIdx]), 0.35f);
                    break;
                }
            }
        }
    }

    void SimulationView::drawVelocityVectors(const model::FluidGrid &grid) const
    {
        const float cellW = static_cast<float>(screenWidth_) / static_cast<float>(grid.width());
        const float cellH = static_cast<float>(screenHeight_) / static_cast<float>(grid.height());
        constexpr int stride = 4;

        for (int gy = 2; gy < grid.height(); gy += stride)
        {
            for (int gx = 2; gx < grid.width(); gx += stride)
            {
                const std::size_t idx = grid.index(gx, gy);
                if (grid.isSolidAt(idx))
                {
                    continue;
                }
                const float vx = grid.vx[idx];
                const float vy = grid.vy[idx];
                const float mag = std::sqrt(vx * vx + vy * vy);
                if (mag < 0.5f)
                {
                    continue;
                }

                const Vector2 start{
                    (static_cast<float>(gx) - 0.5f) * cellW,
                    (static_cast<float>(gy) - 0.5f) * cellH};
                const float scale = std::min(mag * 0.45f, 18.0f);
                const Vector2 end{
                    start.x + (vx / mag) * scale,
                    start.y + (vy / mag) * scale};
                DrawLineV(start, end, Fade(SKYBLUE, 0.65f));
            }
        }
    }

    void SimulationView::drawBrushPreview(const model::FluidGrid &grid, int brushRadius) const
    {
        const Vector2 mouse = GetMousePosition();
        const float cellW = static_cast<float>(screenWidth_) / static_cast<float>(grid.width());
        const float radiusPx = static_cast<float>(brushRadius) * cellW;
        DrawCircleLinesV(mouse, radiusPx, Fade(RAYWHITE, 0.45f));
    }

    void SimulationView::drawHud(const HudState &hud) const
    {
        DrawRectangle(12, 12, 340, 28, Fade(BLACK, 0.65f));
        char statusLine[128];
        std::snprintf(
            statusLine,
            sizeof(statusLine),
            "FPS: %d | %s | %s",
            GetFPS(),
            viewModeName(hud.viewMode),
            hud.paused ? "PAUSA" : "ATTIVO");
        DrawText(statusLine, 20, 18, 16, RAYWHITE);

        if (!hud.showHelp)
        {
            DrawText("Premi [H] per l'elenco controlli", 16, screenHeight_ - 28, 16, Fade(RAYWHITE, 0.7f));
            return;
        }

        DrawRectangle(12, 46, 430, 215, Fade(BLACK, 0.65f));
        DrawText("CONTROLLI:", 22, 54, 16, GOLD);
        DrawText("Click Sinistro + Trascina: Inietta fumo e spinta", 22, 76, 15, RAYWHITE);
        DrawText("Click Destro + Trascina: Disegna ostacolo solido", 22, 96, 15, RAYWHITE);
        DrawText("Shift + Click Destro: Rimuovi ostacolo", 22, 116, 15, RAYWHITE);
        DrawText("[1..4] Modalita vista | [V] Vettori | [S] Sorgente", 22, 138, 15, RAYWHITE);
        DrawText("[SPAZIO] Pausa | [N] Step singolo | [Rotella] Raggio", 22, 158, 15, RAYWHITE);
        DrawText("[C] Pulisci ostacoli | [R] Reset completo | [H] HUD", 22, 178, 15, RAYWHITE);

        char sourceBuf[64];
        std::snprintf(sourceBuf, sizeof(sourceBuf), "Sorgente continua: %s | Raggio: %d",
                      hud.continuousSource ? "ON" : "OFF", hud.brushRadius);
        DrawText(sourceBuf, 22, 204, 15, SKYBLUE);
    }

} // namespace fluid::view
