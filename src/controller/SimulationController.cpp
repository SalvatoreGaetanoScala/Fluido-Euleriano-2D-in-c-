#include "controller/SimulationController.hpp"

#include <algorithm>
#include <cmath>

namespace fluid::controller
{

    SimulationController::SimulationController(const SimulationConfig &config)
        : config_(config),
          solver_(model::GridConfig{config.gridWidth, config.gridHeight, 1.0f}),
          view_(config.screenWidth, config.screenHeight, solver_.grid())
    {
        hud_.continuousSource = solver_.hasContinuousSource();
    }

    void SimulationController::run()
    {
        SetTargetFPS(120);

        while (!WindowShouldClose())
        {
            handleInput();
            applyMouseInteraction();

            const float frameTime = std::min(GetFrameTime(), config_.maxFrameDelta);
            if (!hud_.paused)
            {
                accumulator_ += frameTime;
                while (accumulator_ >= config_.fixedDeltaTime)
                {
                    updateSimulation(config_.fixedDeltaTime);
                    accumulator_ -= config_.fixedDeltaTime;
                }
            }
            else if (singleStepRequested_)
            {
                updateSimulation(config_.fixedDeltaTime);
                singleStepRequested_ = false;
            }

            view_.render(solver_, hud_);
        }
    }

    void SimulationController::handleInput()
    {
        if (IsKeyPressed(KEY_SPACE))
        {
            hud_.paused = !hud_.paused;
        }
        if (IsKeyPressed(KEY_N))
        {
            hud_.paused = true;
            singleStepRequested_ = true;
        }
        if (IsKeyPressed(KEY_R))
        {
            solver_.reset();
            accumulator_ = 0.0f;
        }
        if (IsKeyPressed(KEY_C))
        {
            solver_.grid().clearObstacles();
        }
        if (IsKeyPressed(KEY_S))
        {
            solver_.setContinuousSource(!solver_.hasContinuousSource());
            hud_.continuousSource = solver_.hasContinuousSource();
        }
        if (IsKeyPressed(KEY_V))
        {
            hud_.showVectorArrows = !hud_.showVectorArrows;
        }
        if (IsKeyPressed(KEY_H))
        {
            hud_.showHelp = !hud_.showHelp;
        }

        if (IsKeyPressed(KEY_ONE))
        {
            hud_.viewMode = view::ViewMode::SmokeDensity;
        }
        else if (IsKeyPressed(KEY_TWO))
        {
            hud_.viewMode = view::ViewMode::VelocityField;
        }
        else if (IsKeyPressed(KEY_THREE))
        {
            hud_.viewMode = view::ViewMode::Vorticity;
        }
        else if (IsKeyPressed(KEY_FOUR))
        {
            hud_.viewMode = view::ViewMode::Pressure;
        }

        const float wheel = GetMouseWheelMove();
        if (wheel != 0.0f)
        {
            const int delta = (wheel > 0.0f) ? 1 : -1;
            hud_.brushRadius = std::clamp(hud_.brushRadius + delta, 2, 30);
        }
    }

    void SimulationController::applyMouseInteraction()
    {
        const Vector2 mousePos = GetMousePosition();
        int gx = 0;
        int gy = 0;
        const bool inside = view_.screenToGrid(mousePos, solver_.grid(), gx, gy);

        if (!inside)
        {
            hasPrevMouseGrid_ = false;
            return;
        }

        const bool leftDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
        const bool rightDown = IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
        const bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

        if (leftDown)
        {
            solver_.addDensityBrush(gx, gy, hud_.brushRadius, 3.0f);
            if (hasPrevMouseGrid_)
            {
                const float dx = static_cast<float>(gx - prevMouseGx_);
                const float dy = static_cast<float>(gy - prevMouseGy_);
                solver_.addVelocityBrush(gx, gy, hud_.brushRadius, dx * 28.0f, dy * 28.0f);
            }
        }

        if (rightDown)
        {
            solver_.setObstacleBrush(gx, gy, hud_.brushRadius, !shiftDown);
        }

        prevMouseGx_ = gx;
        prevMouseGy_ = gy;
        hasPrevMouseGrid_ = true;
    }

    void SimulationController::updateSimulation(float dt)
    {
        solver_.step(dt);
    }

} // namespace fluid::controller
