#pragma once

#include "model/FluidSolver.hpp"
#include "view/SimulationView.hpp"
#include "raylib.h"

namespace fluid::controller
{

    struct SimulationConfig
    {
        int screenWidth{1280};
        int screenHeight{800};
        int gridWidth{160};
        int gridHeight{120};
        float fixedDeltaTime{1.0f / 60.0f};
        float maxFrameDelta{0.05f}; // Evita "spiral of death" su frame lenti
    };

    class SimulationController
    {
    public:
        explicit SimulationController(const SimulationConfig &config);
        void run();

    private:
        void handleInput();
        void applyMouseInteraction();
        void updateSimulation(float dt);

        SimulationConfig config_;
        model::FluidSolver solver_;
        view::SimulationView view_;
        view::HudState hud_;

        float accumulator_{0.0f};
        bool singleStepRequested_{false};
        bool hasPrevMouseGrid_{false};
        int prevMouseGx_{0};
        int prevMouseGy_{0};
    };

} // namespace fluid::controller
