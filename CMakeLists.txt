cmake_minimum_required(VERSION 3.20)
project(Fluid2DMVC VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_SANITIZERS "Abilita AddressSanitizer e UndefinedBehaviorSanitizer" OFF)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

# Libreria statica del Model (senza raylib)
add_library(fluid_model STATIC
    src/model/FluidGrid.cpp
    src/model/FluidSolver.cpp
)
target_include_directories(fluid_model PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Eseguibile principale (View + Controller + main)
add_executable(fluid2d_mvc
    src/main.cpp
    src/view/FluidTexture.cpp
    src/view/SimulationView.cpp
    src/controller/SimulationController.cpp
)
target_include_directories(fluid2d_mvc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(fluid2d_mvc PRIVATE
    fluid_model
    raylib
)

# Warning severi e sanitizers
if(MSVC)
    target_compile_options(fluid_model PRIVATE /W4 /WX)
    target_compile_options(fluid2d_mvc PRIVATE /W4 /WX)
else()
    target_compile_options(fluid_model PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(fluid2d_mvc PRIVATE -Wall -Wextra -Wpedantic -Werror)
    if(ENABLE_SANITIZERS)
        target_compile_options(fluid_model PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -g)
        target_link_options(fluid_model PRIVATE -fsanitize=address,undefined)
        target_compile_options(fluid2d_mvc PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -g)
        target_link_options(fluid2d_mvc PRIVATE -fsanitize=address,undefined)
    endif()
endif()
