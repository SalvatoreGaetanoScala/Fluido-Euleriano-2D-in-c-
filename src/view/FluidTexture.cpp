#include "view/FluidTexture.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

namespace fluid::view
{

    FluidTexture::FluidTexture(int width, int height)
        : width_(std::max(1, width)),
          height_(std::max(1, height)),
          pixels_(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), BLACK)
    {
        image_ = GenImageColor(width_, height_, BLACK);
        texture_ = LoadTextureFromImage(image_);
        SetTextureFilter(texture_, TEXTURE_FILTER_BILINEAR);
        initialized_ = true;
    }

    FluidTexture::~FluidTexture()
    {
        release();
    }

    FluidTexture::FluidTexture(FluidTexture &&other) noexcept
        : width_(other.width_),
          height_(other.height_),
          pixels_(std::move(other.pixels_)),
          image_(other.image_),
          texture_(other.texture_),
          initialized_(other.initialized_)
    {
        other.width_ = 0;
        other.height_ = 0;
        other.image_ = {};
        other.texture_ = {};
        other.initialized_ = false;
    }

    FluidTexture &FluidTexture::operator=(FluidTexture &&other) noexcept
    {
        if (this != &other)
        {
            release();
            width_ = other.width_;
            height_ = other.height_;
            pixels_ = std::move(other.pixels_);
            image_ = other.image_;
            texture_ = other.texture_;
            initialized_ = other.initialized_;

            other.width_ = 0;
            other.height_ = 0;
            other.image_ = {};
            other.texture_ = {};
            other.initialized_ = false;
        }
        return *this;
    }

    void FluidTexture::upload()
    {
        if (!initialized_ || image_.data == nullptr || pixels_.empty())
        {
            return;
        }

        const std::size_t bytes = pixels_.size() * sizeof(Color);
        std::memcpy(image_.data, pixels_.data(), bytes);
        UpdateTexture(texture_, image_.data);
    }

    void FluidTexture::release() noexcept
    {
        if (initialized_)
        {
            if (texture_.id > 0)
            {
                UnloadTexture(texture_);
            }
            if (image_.data != nullptr)
            {
                UnloadImage(image_);
            }
            texture_ = {};
            image_ = {};
            initialized_ = false;
        }
    }

} // namespace fluid::view
