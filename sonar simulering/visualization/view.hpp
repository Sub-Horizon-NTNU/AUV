#pragma once

#include "model.hpp"
#include "raylib.h"

namespace viewer {

inline constexpr int canvas_width = 1440;
inline constexpr int canvas_height = 900;

// Mouse coordinates are mapped from the resizable window to the fixed logical canvas.
struct SonarTexture {
    Texture2D texture{};
    unsigned revision{};
    Channel channel{Channel::Combined};
    double gain_db{};
    double dynamic_range_db{};
};
void render_sonar(const Model &model, SonarTexture &image);
void render_world(const Model &model, RenderTexture2D world);
void draw_interface(Model &model, RenderTexture2D world, Texture2D sonar_image, Vector2 mouse,
                    Font font);

} // namespace viewer
