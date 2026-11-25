#pragma once

#include "layout.hpp"
#include "model.hpp"
#include "raylib.h"

namespace viewer {

struct SonarTexture {
    Texture2D texture{};
    unsigned revision{};
    unsigned estimate_revision{};
    Channel channel{Channel::Combined};
    bool estimated_pool{};
    double gain_db{};
    double dynamic_range_db{};
};
struct CameraInput {
    bool dragging{};
    int button{};
    Vector2 previous{};
};
void update_camera_input(Model &model, const Layout &layout, CameraInput &input, Vector2 mouse);
void render_sonar(const Model &model, SonarTexture &image, int requested_size);
void render_world(const Model &model, RenderTexture2D world, Font font);
void draw_interface(Model &model, RenderTexture2D world, Texture2D sonar_image, Vector2 mouse,
                    Font font, const Layout &layout);

} // namespace viewer
