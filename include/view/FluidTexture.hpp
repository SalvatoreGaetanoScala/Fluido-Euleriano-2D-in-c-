#pragma once

#include "raylib.h"

#include <cstddef>
#include <vector>

namespace fluid::view
{

    // Wrapper RAII per Image + Texture2D in raylib.
    class FluidTexture
    {
    public:
        FluidTexture(int width, int height);
        ~FluidTexture();

        FluidTexture(const FluidTexture &) = delete;
        FluidTexture &operator=(const FluidTexture &) = delete;
        FluidTexture(FluidTexture &&other) noexcept;
        FluidTexture &operator=(FluidTexture &&other) noexcept;

        [[nodiscard]] int width() const noexcept { return width_; }
        [[nodiscard]] int height() const noexcept { return height_; }
        [[nodiscard]] std::vector<Color> &pixels() noexcept { return pixels_; }
        [[nodiscard]] const Texture2D &texture() const noexcept { return texture_; }

        void upload();

    private:
        void release() noexcept;

        int width_{0};
        int height_{0};
        std::vector<Color> pixels_;
        Image image_{};
        Texture2D texture_{};
        bool initialized_{false};
    };

} // namespace fluid::view
