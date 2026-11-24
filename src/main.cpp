#include "controller/SimulationController.hpp"
#include "raylib.h"

int main()
{
    fluid::controller::SimulationConfig config{};
    config.screenWidth = 1280;
    config.screenHeight = 800;
    config.gridWidth = 160;
    config.gridHeight = 120;
    config.fixedDeltaTime = 1.0f / 60.0f;

    InitWindow(config.screenWidth, config.screenHeight,
               "Fluido Euleriano 2D - C++17 + raylib (MVC)");

    {
        fluid::controller::SimulationController controller(config);
        controller.run();
    } // controller e FluidTexture vengono distrutti prima di CloseWindow()

    CloseWindow();
    return 0;
}
