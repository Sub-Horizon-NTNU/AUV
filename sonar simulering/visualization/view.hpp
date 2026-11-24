#pragma once

#include "model.hpp"
#include "raylib.h"

namespace viewer {

inline constexpr int canvas_width = 1440;
inline constexpr int canvas_height = 900;

// Mouse coordinates are mapped from the resizable window to the fixed logical canvas.
void render_world(const Model &model, RenderTexture2D world);
void draw_interface(Model &model, RenderTexture2D world, Vector2 mouse, Font font);

} // namespace viewer
