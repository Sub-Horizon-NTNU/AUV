#include "view.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace viewer {
namespace {

constexpr Color background{13, 20, 30, 255};
constexpr Color panel{23, 34, 48, 255};
constexpr Color text{231, 239, 246, 255};
constexpr Color muted{161, 181, 200, 255};
constexpr Color cyan{74, 221, 230, 255};
constexpr Color amber{255, 192, 87, 255};
constexpr Color ghost{216, 147, 255, 255};
constexpr Color green{129, 228, 149, 255};
constexpr Rectangle sonar_image_area{566, 172, 316, 316};

std::string number(double value, int precision = 2) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

void label(Font font, const std::string &value, float x, float y, int size = 18,
           Color color = text) {
    DrawTextEx(font, value.c_str(), {x, y}, static_cast<float>(std::max(18, size)), 0.5F, color);
}

void wrapped(Font font, const std::string &value, float x, float y, float width, int size,
             Color color) {
    size = std::max(18, size);
    std::istringstream words(value);
    std::string word;
    std::string line;
    while (words >> word) {
        const std::string candidate = line.empty() ? word : line + ' ' + word;
        if (!line.empty() &&
            static_cast<float>(
                MeasureTextEx(font, candidate.c_str(), static_cast<float>(size), 0.5F).x) > width) {
            label(font, line, x, y, size, color);
            y += static_cast<float>(size + 5);
            line = word;
        } else {
            line = candidate;
        }
    }
    label(font, line, x, y, size, color);
}

bool button(Font font, const char *caption, Rectangle area, Vector2 mouse, bool active = false) {
    const bool hovered = CheckCollisionPointRec(mouse, area);
    DrawRectangleRounded(area, 0.15F, 4,
                         active    ? Color{48, 93, 108, 255}
                         : hovered ? Color{55, 72, 91, 255}
                                   : Color{38, 53, 70, 255});
    DrawRectangleRoundedLinesEx(area, 0.15F, 4, 1.0F, active ? cyan : Color{67, 87, 107, 255});
    const float width = MeasureTextEx(font, caption, 18, 0.5F).x;
    label(font, caption, area.x + (area.width - static_cast<float>(width)) / 2.0F,
          area.y + (area.height - 18.0F) / 2.0F, 18);
    return hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

double slider(Rectangle area, double value, Vector2 mouse, Color color) {
    DrawRectangleRounded({area.x, area.y + area.height / 2 - 2, area.width, 4}, 0.5F, 4,
                         Color{72, 91, 111, 255});
    DrawCircleV({area.x + static_cast<float>(value) * area.width, area.y + area.height / 2}, 7,
                color);
    if (CheckCollisionPointRec(mouse, area) && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        return std::clamp(static_cast<double>((mouse.x - area.x) / area.width), 0.0, 1.0);
    }
    return value;
}

Vector3 point(sonar::Vec3 value) {
    // Core pool coordinates are z-up; raylib's world is y-up.
    return {static_cast<float>(value.x), static_cast<float>(value.z), static_cast<float>(value.y)};
}

void path3d(const sonar::Echo &echo, Color color) {
    for (std::size_t index = 1; index <= echo.outward_hits; ++index) {
        DrawLine3D(point(echo.path[index - 1].position_m), point(echo.path[index].position_m),
                   color);
    }
}

void arrow2d(Vector2 start, Vector2 end, Color color, float thickness = 2.0F) {
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float distance = std::hypot(dx, dy);
    if (distance < 2.0F) {
        return;
    }
    const float ux = dx / distance;
    const float uy = dy / distance;
    const float head = std::min(9.0F, distance * 0.3F);
    DrawLineEx(start, end, thickness, color);
    DrawLineEx(end, {end.x - ux * head - uy * head * 0.5F, end.y - uy * head + ux * head * 0.5F},
               thickness, color);
    DrawLineEx(end, {end.x - ux * head + uy * head * 0.5F, end.y - uy * head - ux * head * 0.5F},
               thickness, color);
}

bool direction_sample(const WavefrontPoint &sample, const GeometricWavefront &front) {
    // A few vectors explain motion; never draw arrows on interpolated mesh seams.
    const unsigned h = sample.horizontal_index;
    const unsigned v = sample.vertical_index;
    return sample.ray_id < front.rays.size() &&
           (h == 0 || h == front.horizontal_rays / 2 || h + 1 == front.horizontal_rays) &&
           (v == 0 || v == front.vertical_rays / 4 || v == front.vertical_rays / 2 ||
            v == 3 * front.vertical_rays / 4 || v + 1 == front.vertical_rays);
}

void draw_world3d(const Model &model, RenderTexture2D target, Font font) {
    BeginTextureMode(target);
    ClearBackground(background);
    const auto &pool = model.config.pool;
    const float marker_scale =
        static_cast<float>(std::max({pool.length_m, pool.width_m, pool.depth_m}));
    // Keep the physical pool legible; apparent echoes may extend beyond the viewport.
    const float extent = marker_scale;
    const Vector3 center = point({pool.length_m / 2, pool.width_m / 2, pool.depth_m / 2});
    const Vector3 pool_center = point({pool.length_m / 2, pool.width_m / 2, pool.depth_m / 2});
    const float azimuth = model.orbit_deg * DEG2RAD;
    const float elevation = model.camera_pitch_deg * DEG2RAD;
    const float distance = extent * 1.55F * model.camera_zoom;
    Camera3D camera{};
    camera.position = {center.x + std::cos(azimuth) * std::cos(elevation) * distance,
                       center.y + std::sin(elevation) * distance,
                       center.z + std::sin(azimuth) * std::cos(elevation) * distance};
    camera.target = center;
    camera.up = {0, 1, 0};
    camera.fovy = 43;
    camera.projection = CAMERA_PERSPECTIVE;
    BeginMode3D(camera);
    DrawCube({pool_center.x, 0, pool_center.z}, static_cast<float>(pool.length_m), 0.01F,
             static_cast<float>(pool.width_m), Color{30, 49, 64, 255});
    DrawCubeWires(pool_center, static_cast<float>(pool.length_m), static_cast<float>(pool.depth_m),
                  static_cast<float>(pool.width_m), Color{110, 145, 168, 255});
    const Vector3 surface_center{pool_center.x, static_cast<float>(pool.depth_m), pool_center.z};
    DrawCubeWires(surface_center, static_cast<float>(pool.length_m), 0.001F,
                  static_cast<float>(pool.width_m), Color{49, 125, 153, 255});
    if (!model.wavefront_mode) {
        for (const auto &echo : model.ping.echoes) {
            if (echo.outward_hits == model.config.settings.max_outward_hits ||
                echo.outward_hits == 1) {
                path3d(echo, Color{59, 77, 92, 255});
            }
        }
    }
    if (model.show_particle_markers)
        for (const auto &particle : model.particles) {
            DrawPoint3D(point(particle), Color{66, 106, 80, 255});
        }
    DrawSphere(point(model.config.pose.position_m), marker_scale * 0.018F, cyan);
    if (const auto *echo = model.echo(); echo && !model.wavefront_mode) {
        path3d(*echo, amber);
        const auto terminal = echo->path[echo->outward_hits].position_m;
        DrawSphereWires(point(terminal), marker_scale * 0.025F, 8, 8, amber);
        const auto apparent = apparent_position(*echo);
        DrawLine3D(point(echo->path.front().position_m), point(apparent), Color{110, 72, 132, 255});
        DrawSphereWires(point(apparent), marker_scale * 0.025F, 8, 8, ghost);
        DrawSphere(point(path_position(*echo, model.progress)), marker_scale * 0.018F,
                   model.progress <= 0.5 ? cyan : amber);
    }
    if (model.wavefront_mode) {
        const auto front =
            model.wavefront.sample_at_time(model.progress * model.wavefront.duration_s());
        for (const auto &link : front.links) {
            DrawLine3D(point(front.points[link.first].position_m),
                       point(front.points[link.second].position_m),
                       front.points[link.first].segment_index == 0 &&
                               front.points[link.second].segment_index == 0
                           ? cyan
                           : amber);
        }
        for (const auto &sample : front.points) {
            DrawPoint3D(point(sample.position_m), sample.segment_index == 0 ? cyan : amber);
            if (direction_sample(sample, model.wavefront)) {
                const double size = marker_scale * 0.065;
                const auto tip = sample.position_m + sample.direction * size;
                const auto head = sample.position_m + sample.direction * (size * 0.7);
                const Color color = sample.segment_index == 0 ? cyan : amber;
                DrawLine3D(point(sample.position_m), point(tip), color);
                DrawCylinderEx(point(head), point(tip), static_cast<float>(size * 0.09), 0, 8,
                               color);
            }
        }
    }
    EndMode3D();
    // A screen-anchored triad shows the posed SONAR axes, not the pool axes.
    const Vector2 origin{72, 72};
    const sonar::Vec3 axes[]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const Color colors[]{RED, GREEN, BLUE};
    const char *names[]{"+X", "+Y", "+Z"};
    for (unsigned index = 0; index < 3; ++index) {
        const auto endpoint = point(model.config.pose.position_m +
                                    model.sonar_to_pool_direction(axes[index]) * extent * 0.2);
        const auto base = GetWorldToScreenEx(point(model.config.pose.position_m), camera,
                                             target.texture.width, target.texture.height);
        const auto projected =
            GetWorldToScreenEx(endpoint, camera, target.texture.width, target.texture.height);
        const float dx = projected.x - base.x;
        const float dy = projected.y - base.y;
        const float magnitude = std::hypot(dx, dy);
        if (magnitude > 1e-4F) {
            const Vector2 end{origin.x + dx / magnitude * 34, origin.y + dy / magnitude * 34};
            arrow2d(origin, end, colors[index]);
            label(font, names[index], end.x + 4, end.y - 8, 16, colors[index]);
        }
    }
    label(font, "SONAR axes", 14, 12, 16, muted);
    EndTextureMode();
}

void draw_world2d(const Model &model, Font font, Rectangle world_area) {
    const bool side = model.projection == Projection::Side;
    const auto &pool = model.config.pool;
    const bool aligned = model.sonar_aligned && !side;
    const auto coordinates = [&](sonar::Vec3 value) {
        if (aligned) {
            const auto local = model.pool_to_sonar(value);
            return Vector2{static_cast<float>(-local.y), static_cast<float>(local.x)};
        }
        return Vector2{static_cast<float>(value.x), static_cast<float>(side ? value.z : value.y)};
    };
    std::vector<sonar::Vec3> corners;
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
    for (unsigned index = 0; index < 8; ++index) {
        const sonar::Vec3 corner{(index & 1) ? pool.length_m : 0, (index & 2) ? pool.width_m : 0,
                                 (index & 4) ? pool.depth_m : 0};
        corners.push_back(corner);
        const auto value = coordinates(corner);
        xmin = std::min(xmin, static_cast<double>(value.x));
        xmax = std::max(xmax, static_cast<double>(value.x));
        ymin = std::min(ymin, static_cast<double>(value.y));
        ymax = std::max(ymax, static_cast<double>(value.y));
    }
    if (const auto *echo = model.echo()) {
        const auto apparent = coordinates(apparent_position(*echo));
        xmin = std::min(xmin, static_cast<double>(apparent.x));
        xmax = std::max(xmax, static_cast<double>(apparent.x));
        ymin = std::min(ymin, static_cast<double>(apparent.y));
        ymax = std::max(ymax, static_cast<double>(apparent.y));
    }
    if (aligned) {
        // Same receiver-centered origin as the sonar image; fit both sides equally.
        xmax = std::max(std::abs(xmin), std::abs(xmax));
        ymax = std::max(std::abs(ymin), std::abs(ymax));
        xmin = -xmax;
        ymin = -ymax;
    }
    const float scale = std::min((world_area.width - 50) / static_cast<float>(xmax - xmin),
                                 (world_area.height - 50) / static_cast<float>(ymax - ymin));
    const float left =
        world_area.x + (world_area.width - static_cast<float>(xmax - xmin) * scale) / 2;
    const float bottom =
        world_area.y + (world_area.height + static_cast<float>(ymax - ymin) * scale) / 2 - 15;
    const auto project = [&](sonar::Vec3 value) {
        const auto value2d = coordinates(value);
        return Vector2{left + (value2d.x - static_cast<float>(xmin)) * scale,
                       bottom - (value2d.y - static_cast<float>(ymin)) * scale};
    };
    for (unsigned index = 0; index < 8; ++index) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            const unsigned neighbor = index ^ (1U << axis);
            if (neighbor > index) {
                DrawLineEx(project(corners[index]), project(corners[neighbor]), 2,
                           Color{104, 146, 171, 255});
            }
        }
    }
    label(font,
          aligned ? "SONAR frame / +X up, +Y left"
          : side  ? "POOL frame / X right, Z up"
                  : "POOL frame / X right, Y up",
          world_area.x + 14, world_area.y + 12, 16, muted);
    const Vector2 axis_origin{world_area.x + 85, world_area.y + 85};
    const Vector2 axis_x{axis_origin.x + (aligned ? 0 : 40), axis_origin.y - (aligned ? 40 : 0)};
    const Vector2 axis_y{axis_origin.x - (aligned ? 40 : 0), axis_origin.y - (aligned ? 0 : 40)};
    arrow2d(axis_origin, axis_x, RED);
    arrow2d(axis_origin, axis_y, side ? BLUE : GREEN);
    label(font, "+X", axis_x.x + 4, axis_x.y - 18, 16, RED);
    label(font, side ? "+Z" : "+Y", axis_y.x - 24, axis_y.y - 18, 16, side ? BLUE : GREEN);
    if (model.show_particle_markers) {
        for (const auto &particle : model.particles) {
            DrawPixelV(project(particle), Color{66, 106, 80, 255});
        }
    }
    for (const auto &echo : model.ping.echoes) {
        for (std::size_t index = 1; index <= echo.outward_hits; ++index) {
            DrawLineV(project(echo.path[index - 1].position_m),
                      project(echo.path[index].position_m), Color{53, 74, 90, 255});
        }
    }
    if (const auto *echo = model.echo(); echo && !model.wavefront_mode) {
        for (std::size_t index = 1; index <= echo->outward_hits; ++index) {
            DrawLineEx(project(echo->path[index - 1].position_m),
                       project(echo->path[index].position_m), 3, amber);
            DrawCircleV(project(echo->path[index].position_m), 4, amber);
        }
        DrawLineEx(project(echo->path.front().position_m), project(apparent_position(*echo)), 1,
                   ghost);
        DrawCircleLinesV(project(apparent_position(*echo)), 8, ghost);
        DrawCircleV(project(path_position(*echo, model.progress)), 6,
                    model.progress <= 0.5 ? cyan : amber);
        DrawCircleLinesV(project(echo->path[echo->outward_hits].position_m), 8, amber);
    }
    if (model.wavefront_mode) {
        const auto front =
            model.wavefront.sample_at_time(model.progress * model.wavefront.duration_s());
        for (const auto &link : front.links) {
            DrawLineEx(project(front.points[link.first].position_m),
                       project(front.points[link.second].position_m), 2,
                       front.points[link.first].segment_index == 0 &&
                               front.points[link.second].segment_index == 0
                           ? cyan
                           : amber);
        }
        for (const auto &sample : front.points) {
            DrawCircleV(project(sample.position_m), 3, sample.segment_index == 0 ? cyan : amber);
            if (direction_sample(sample, model.wavefront)) {
                const auto start = project(sample.position_m);
                const auto projected = project(sample.position_m + sample.direction);
                const float dx = projected.x - start.x;
                const float dy = projected.y - start.y;
                const float magnitude = std::hypot(dx, dy);
                if (magnitude > 1e-4F) {
                    arrow2d(start, {start.x + dx / magnitude * 26, start.y + dy / magnitude * 26},
                            sample.segment_index == 0 ? cyan : amber, 2.5F);
                }
            }
        }
    }
    DrawCircleV(project(model.config.pose.position_m), 6, cyan);
}

void inspector(Model &model, Vector2 mouse, Font font) {
    DrawRectangleRounded({978, 78, 438, 365}, 0.04F, 4, panel);
    label(font, "2  INSPECT AN ECHO", 998, 97, 20);
    if (button(font, "<", {998, 130, 36, 32}, mouse)) {
        model.cycle_echo(-1);
    }
    if (button(font, ">", {1358, 130, 36, 32}, mouse)) {
        model.cycle_echo(1);
    }
    label(font,
          "Echo " + std::to_string(model.ping.echoes.empty() ? 0 : model.selected + 1) + " / " +
              std::to_string(model.ping.echoes.size()) + "  [Up / Down]",
          1046, 138, 17, muted);
    if (const auto *echo = model.echo()) {
        label(font,
              echo->kind == sonar::EchoKind::Volume ? "VOLUME  /  particle in the water"
              : echo->is_multipath()                ? "MULTIPATH  /  reflected via more surfaces"
                                                    : "DIRECT  /  first surface contact",
              998, 178, 17,
              echo->kind == sonar::EchoKind::Volume ? green
              : echo->is_multipath()                ? amber
                                                    : cyan);
        label(font, "Total sound path", 998, 211, 18, muted);
        label(font, number(echo->total_path_m) + " m", 1260, 211, 20);
        label(font, "Displayed range = path / 2", 998, 241, 18, muted);
        label(font, number(echo->apparent_range_m) + " m", 1260, 241, 20, ghost);
        label(font, "Return time", 998, 271, 18, muted);
        label(font, number(echo->travel_time_s * 1000) + " ms", 1260, 271, 20);
        label(font,
              (echo->kind == sonar::EchoKind::Volume
                   ? "Particle " + std::to_string(echo->scatterer_id)
                   : "Ray " + std::to_string(echo->ray_id)) +
                  "  |  " + std::to_string(echo->interaction_count) +
                  " roundtrip contact locations",
              998, 305, 17, muted);
        wrapped(font, "Outward contacts: " + contacts(*echo), 998, 333, 398, 16, text);
        label(font, "The sonar divides the roundtrip by two:", 998, 375, 16, muted);
        label(font, number(echo->total_path_m) + " / 2 = " + number(echo->apparent_range_m) + " m",
              998, 394, 16, ghost);
        label(font,
              echo->in_receive_window ? "Arrival is inside the receive window."
                                      : "Outside receive window: geometry only.",
              998, 410, 16, echo->in_receive_window ? muted : amber);
    } else {
        wrapped(font,
                "No eligible echo in this channel at this angle. Try All, another angle, more "
                "range or enable particles.",
                998, 191, 390, 20, amber);
    }
}

void configuration(Model &model, Vector2 mouse, Font font) {
    DrawRectangleRounded({978, 457, 438, 406}, 0.04F, 4, panel);
    label(font, "3  CONFIGURATION", 998, 475, 20);
    if (button(font, model.scattering_controls ? "Pool >" : "Scattering >", {1232, 469, 160, 32},
               mouse)) {
        model.scattering_controls = !model.scattering_controls;
    }
    label(font,
          model.scattering_controls ? "Surface / water scattering approximations"
                                    : "Apply draft together. Distances in metres.",
          998, 505, 16, muted);
    if (!model.scattering_controls) {
        struct Field {
            const char *name;
            double *value;
            double step;
        };
        Field fields[]{{"Pool length x", &model.draft.pool.length_m, 0.5},
                       {"Pool width y", &model.draft.pool.width_m, 0.5},
                       {"Water depth z", &model.draft.pool.depth_m, 0.25},
                       {"Sonar x", &model.draft.pose.position_m.x, 0.25},
                       {"Sonar y", &model.draft.pose.position_m.y, 0.25},
                       {"Sonar z (above floor)", &model.draft.pose.position_m.z, 0.25},
                       {"Receive range", &model.draft.settings.max_range_m, 1.0}};
        float y = 535;
        for (auto &field : fields) {
            label(font, field.name, 998, y + 5, 17, muted);
            if (button(font, "-", {1238, y, 30, 26}, mouse)) {
                *field.value -= field.step;
            }
            label(font, number(*field.value), 1276, y + 5, 17);
            if (button(font, "+", {1362, y, 30, 26}, mouse)) {
                *field.value += field.step;
            }
            y += 32;
        }
        label(font, "Max outward contacts", 998, y + 5, 17, muted);
        if (button(font, "-", {1238, y, 30, 26}, mouse) &&
            model.draft.settings.max_outward_hits > 1) {
            --model.draft.settings.max_outward_hits;
        }
        label(font, std::to_string(model.draft.settings.max_outward_hits), 1295, y + 5, 17);
        if (button(font, "+", {1362, y, 30, 26}, mouse) &&
            model.draft.settings.max_outward_hits < 16) {
            ++model.draft.settings.max_outward_hits;
        }
    } else {
        auto &settings = model.draft.settings;
        const bool rough = settings.surface_model == sonar::SurfaceModel::Roughness;
        if (button(font, rough ? "Surface: roughness model" : "Surface: legacy backscatter",
                   {998, 538, 394, 32}, mouse)) {
            settings.surface_model =
                rough ? sonar::SurfaceModel::Legacy : sonar::SurfaceModel::Roughness;
        }
        label(font, "RMS roughness (mm)", 998, 587, 17, muted);
        if (button(font, "-", {1238, 580, 30, 28}, mouse)) {
            settings.surface_roughness_m -= 0.00005;
        }
        label(font, number(settings.surface_roughness_m * 1000, 2), 1276, 587, 17);
        if (button(font, "+", {1362, 580, 30, 28}, mouse)) {
            settings.surface_roughness_m += 0.00005;
        }
        if (button(font, settings.volume_scatterers ? "Particles: enabled" : "Particles: disabled",
                   {998, 619, 394, 32}, mouse)) {
            settings.volume_scatterers = settings.volume_scatterers ? 0 : 1024;
        }
        label(font, "Particle samples", 998, 668, 17, muted);
        if (button(font, "-", {1238, 661, 30, 28}, mouse)) {
            settings.volume_scatterers =
                settings.volume_scatterers > 256 ? settings.volume_scatterers - 256 : 0;
        }
        label(font, std::to_string(settings.volume_scatterers), 1276, 668, 17);
        if (button(font, "+", {1362, 661, 30, 28}, mouse)) {
            settings.volume_scatterers = std::min(4096U, settings.volume_scatterers + 256);
        }
        label(font, "Particle strength (relative)", 998, 710, 16, muted);
        if (button(font, "-", {1238, 703, 30, 28}, mouse)) {
            settings.volume_strength = std::max(0.0, settings.volume_strength - 0.001);
        }
        label(font, number(settings.volume_strength, 3), 1276, 710, 17);
        if (button(font, "+", {1362, 703, 30, 28}, mouse)) {
            settings.volume_strength = std::min(1.0, settings.volume_strength + 0.001);
        }
        wrapped(font, "Illustrative surface / particle model. Not calibrated to a pool.", 998, 749,
                394, 18, muted);
    }
    if (button(font, "Apply draft", {998, 807, 180, 36}, mouse)) {
        model.apply(model.draft, model.ping.angle_grad);
    }
    if (button(font, "Reload config", {1192, 807, 200, 36}, mouse)) {
        model.reload();
    }
}

void sonar_panel(Model &model, Texture2D texture, Vector2 mouse, Font font) {
    DrawRectangleRec({494, 112, 460, 410}, panel);
    const char *names[]{"All", "Direct", "Multipath", "Particles"};
    for (unsigned index = 0; index < 4; ++index) {
        const auto channel = static_cast<Channel>(index);
        if (button(font, names[index], {504 + static_cast<float>(index) * 110, 122, 104, 28}, mouse,
                   model.channel == channel)) {
            model.set_channel(channel);
        }
    }
    DrawTexturePro(texture,
                   {0, 0, static_cast<float>(texture.width), static_cast<float>(texture.height)},
                   sonar_image_area, {0, 0}, 0, WHITE);
    const Vector2 center{sonar_image_area.x + sonar_image_area.width / 2,
                         sonar_image_area.y + sonar_image_area.height / 2};
    const float radius = sonar_image_area.width / 2;
    for (int ring = 1; ring <= 4; ++ring) {
        DrawCircleLinesV(center, radius * static_cast<float>(ring) / 4, Color{66, 88, 111, 160});
    }
    const float angle = static_cast<float>(model.ping.angle_grad) * 0.9F * DEG2RAD;
    DrawLineV(center, {center.x - std::sin(angle) * radius, center.y - std::cos(angle) * radius},
              Color{200, 214, 226, 150});
    if (const auto *echo = model.echo()) {
        const float r =
            static_cast<float>(echo->apparent_range_m / model.config.settings.max_range_m) * radius;
        DrawCircleLinesV({center.x - std::sin(angle) * r, center.y - std::cos(angle) * r}, 5,
                         WHITE);
    }
    label(font, "+x / 0 grad", center.x - 37, 153, 16, muted);
    label(font, "+y", 530, center.y - 10, 16, muted);
    label(font, "-y", 897, center.y - 10, 16, muted);
    const Vector2 axes_origin{sonar_image_area.x + 62, sonar_image_area.y + 64};
    const Vector2 axes_x{axes_origin.x, axes_origin.y - 30};
    const Vector2 axes_y{axes_origin.x - 30, axes_origin.y};
    arrow2d(axes_origin, axes_x, RED);
    arrow2d(axes_origin, axes_y, GREEN);
    label(font, "+X", axes_x.x + 5, axes_x.y - 12, 16, RED);
    label(font, "+Y", axes_y.x - 23, axes_y.y - 16, 16, GREEN);
    label(font, "Local frame; outer ring " + number(model.config.settings.max_range_m, 1) + " m",
          566, 475, 16, muted);
    if (CheckCollisionPointRec(mouse, sonar_image_area) &&
        IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        model.select_image((center.y - mouse.y) / radius, (center.x - mouse.x) / radius);
    }
    if (button(font, "-", {505, 494, 26, 22}, mouse)) {
        model.gain_db = std::max(-40.0, model.gain_db - 5);
    }
    label(font, "Gain " + number(model.gain_db, 0) + " dB", 538, 498, 14, muted);
    if (button(font, "+", {630, 494, 26, 22}, mouse)) {
        model.gain_db = std::min(60.0, model.gain_db + 5);
    }
    if (button(font, "-", {689, 494, 26, 22}, mouse)) {
        model.dynamic_range_db = std::max(20.0, model.dynamic_range_db - 10);
    }
    label(font, "Span " + number(model.dynamic_range_db, 0) + " dB", 722, 498, 14, muted);
    if (button(font, "+", {918, 494, 26, 22}, mouse)) {
        model.dynamic_range_db = std::min(120.0, model.dynamic_range_db + 10);
    }
}

} // namespace

void render_world(const Model &model, RenderTexture2D world, Font font) {
    draw_world3d(model, world, font);
}

void render_sonar(const Model &model, SonarTexture &image, int requested_size) {
    const int size = std::clamp(requested_size, 384, 2048);
    if (image.texture.id && image.texture.width == size && image.revision == model.scan_revision &&
        image.channel == model.channel && image.gain_db == model.gain_db &&
        image.dynamic_range_db == model.dynamic_range_db) {
        return;
    }
    Image pixels = GenImageColor(size, size, background);
    auto *colors = static_cast<Color *>(pixels.data);
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const double forward = (size / 2.0 - y - 0.5) / (size / 2.0);
            const double lateral = (size / 2.0 - x - 0.5) / (size / 2.0);
            const double radius = std::hypot(forward, lateral);
            if (radius >= 1) {
                continue;
            }
            double angle = std::atan2(lateral, forward) * 180 / 3.14159265358979323846;
            if (angle < 0) {
                angle += 360;
            }
            const auto heading = static_cast<unsigned>(std::lround(angle / 0.9)) % 400;
            const auto &bins = model.scan[heading];
            const int index = static_cast<int>(radius * static_cast<double>(bins.size()));
            const int footprint = std::max(1, static_cast<int>(bins.size()) / (size / 2));
            double energy = 0;
            // Max resampling preserves thin range returns on the lower-resolution display.
            for (int bin = std::max(0, index - footprint);
                 bin <= std::min(static_cast<int>(bins.size()) - 1, index + footprint); ++bin) {
                energy =
                    std::max(energy, model.channel_energy(bins[static_cast<std::size_t>(bin)]));
            }
            const auto brightness = static_cast<unsigned char>(255 * model.display_level(energy));
            colors[y * size + x] = {static_cast<unsigned char>(brightness / 3), brightness,
                                    static_cast<unsigned char>(brightness * 0.9), 255};
        }
    }
    if (image.texture.id) {
        UnloadTexture(image.texture);
    }
    image.texture = LoadTextureFromImage(pixels);
    UnloadImage(pixels);
    SetTextureFilter(image.texture, TEXTURE_FILTER_BILINEAR);
    image.revision = model.scan_revision;
    image.channel = model.channel;
    image.gain_db = model.gain_db;
    image.dynamic_range_db = model.dynamic_range_db;
}

void update_camera_input(Model &model, const Layout &layout, CameraInput &input, Vector2 mouse) {
    const bool over_drawer =
        (model.settings_open || model.details_open) &&
        CheckCollisionPointRec(mouse, {layout.configuration.x, layout.configuration.y,
                                       layout.configuration.width, layout.configuration.height});
    if (model.help_open || model.projection != Projection::Perspective) {
        input.dragging = false;
        model.camera_dragging = false;
        return;
    }
    if (input.dragging) {
        if (!IsMouseButtonDown(input.button)) {
            input.dragging = false;
        } else {
            model.orbit_camera(-(mouse.x - input.previous.x) / layout.world.width * 180,
                               (mouse.y - input.previous.y) / layout.world.height * 120);
            input.previous = mouse;
        }
    } else if (!over_drawer &&
               CheckCollisionPointRec(mouse, {layout.world.x, layout.world.y, layout.world.width,
                                              layout.world.height})) {
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            input.dragging = true;
            input.button =
                IsMouseButtonPressed(MOUSE_BUTTON_LEFT) ? MOUSE_BUTTON_LEFT : MOUSE_BUTTON_RIGHT;
            input.previous = mouse;
        }
    }
    if (!over_drawer && CheckCollisionPointRec(mouse, {layout.world.x, layout.world.y,
                                                       layout.world.width, layout.world.height})) {
        model.zoom_camera(GetMouseWheelMove());
    }
    model.camera_dragging = input.dragging;
}

void draw_interface(Model &model, RenderTexture2D world, Texture2D sonar_image, Vector2 mouse,
                    Font font, const Layout &layout) {
    ClearBackground(background);
    const float s = layout.scale;
    const auto box = [](Box b) { return Rectangle{b.x, b.y, b.width, b.height}; };
    const auto text_at = [&](const std::string &value, float x, float y, int size = 18,
                             Color color = text) {
        label(font, value, x, y, std::max(18, static_cast<int>(static_cast<float>(size) * s)),
              color);
    };
    Vector2 active_mouse =
        model.help_open || model.camera_dragging ? Vector2{-10000, -10000} : mouse;
    const float h = std::max(25.0F, 29 * s);
    text_at("PING360 / FOLLOW THE SOUND", layout.header.x, layout.header.y, 25);
    text_at("Compare physical paths with apparent sonar returns", layout.header.x,
            layout.header.y + 31 * s, 15, muted);
    float header_x = layout.header.x + layout.header.width - 380 * s;
    if (button(font, "Profile", {header_x, layout.header.y, 85 * s, h}, active_mouse,
               model.profile_open)) {
        model.profile_open = !model.profile_open;
    }
    header_x += 94 * s;
    if (button(font, "Details", {header_x, layout.header.y, 85 * s, h}, active_mouse,
               model.details_open)) {
        model.details_open = !model.details_open;
        model.settings_open = false;
    }
    header_x += 94 * s;
    if (button(font, "Settings", {header_x, layout.header.y, 85 * s, h}, active_mouse,
               model.settings_open)) {
        model.settings_open = !model.settings_open;
        model.details_open = false;
    }
    header_x += 94 * s;
    if (button(font, "Help [H]", {header_x, layout.header.y, 95 * s, h}, active_mouse)) {
        model.help_open = true;
    }
    if (IsKeyPressed(KEY_H)) {
        model.help_open = !model.help_open;
    }
    const bool drawer = model.settings_open || model.details_open;
    const bool over_drawer = drawer && CheckCollisionPointRec(mouse, box(layout.configuration));
    const Vector2 content_mouse =
        model.help_open || model.camera_dragging || over_drawer ? Vector2{-10000, -10000} : mouse;
    if (!model.help_open) {
        if (IsKeyPressed(KEY_LEFT)) {
            model.change_angle(-1);
        }
        if (IsKeyPressed(KEY_RIGHT)) {
            model.change_angle(1);
        }
        if (IsKeyPressed(KEY_UP)) {
            model.cycle_echo(-1);
        }
        if (IsKeyPressed(KEY_DOWN)) {
            model.cycle_echo(1);
        }
        if (IsKeyPressed(KEY_SPACE)) {
            model.playing = !model.playing;
        }
    }
    const auto ray = layout.ray_panel;
    DrawRectangleRec(box(ray), panel);
    text_at("RAY", ray.x + 10 * s, ray.y + 8 * s, 18);
    if (button(font, model.sonar_aligned ? "Pool frame" : "Align sonar",
               {ray.x + 65 * s, ray.y + 5 * s, 130 * s, h}, content_mouse, model.sonar_aligned)) {
        if (model.sonar_aligned) {
            model.sonar_aligned = false;
        } else {
            model.align_with_sonar();
        }
    }
    float x = ray.x + ray.width - 245 * s;
    const char *views[] = {"3D", "Top", "Side", "Reset"};
    for (int i = 0; i < 4; ++i) {
        if (button(font, views[i], {x, ray.y + 5 * s, 54 * s, h}, content_mouse,
                   i < 3 && static_cast<int>(model.projection) == i)) {
            if (i < 3) {
                model.projection = static_cast<Projection>(i);
                if (i != 1) {
                    model.sonar_aligned = false;
                }
            } else {
                model.reset_camera();
            }
        }
        x += 60 * s;
    }
    if (button(font, "Ray paths", {ray.x + 10 * s, ray.y + 36 * s, 100 * s, h}, content_mouse,
               !model.wavefront_mode)) {
        model.wavefront_mode = false;
        model.progress = 0;
    }
    if (button(font, "Wavefront", {ray.x + 118 * s, ray.y + 36 * s, 110 * s, h}, content_mouse,
               model.wavefront_mode)) {
        model.wavefront_mode = true;
        model.progress = 0;
    }
    if (button(font, model.show_particle_markers ? "Particle markers: on" : "Particle markers: off",
               {ray.x + 236 * s, ray.y + 36 * s, 195 * s, h}, content_mouse,
               model.show_particle_markers)) {
        model.show_particle_markers = !model.show_particle_markers;
    }
    const auto area = layout.world;
    BeginScissorMode(static_cast<int>(area.x), static_cast<int>(area.y),
                     static_cast<int>(area.width), static_cast<int>(area.height));
    if (model.projection == Projection::Perspective) {
        DrawTexturePro(world.texture,
                       {0, 0, static_cast<float>(world.texture.width),
                        -static_cast<float>(world.texture.height)},
                       box(area), {0, 0}, 0, WHITE);
    } else {
        draw_world2d(model, font, box(area));
    }
    EndScissorMode();
    text_at(model.wavefront_mode ? "Arrows: cyan incident / amber reflected"
                                 : "Amber: physical path   Purple: apparent position",
            ray.x + 10 * s, ray.y + ray.height - 44 * s, 14, muted);
    text_at(model.sonar_aligned ? "SONAR frame: +X up / +Y left, matching image"
                                : "Drag in 3D to orbit; Align sonar matches image",
            ray.x + 10 * s, ray.y + ray.height - 22 * s, 13, muted);
    // Retain established controls in an independent native vector panel, not a full-window bitmap.
    const auto draw_panel = [&](Rectangle source, Box destination, const auto &draw,
                                Vector2 input) {
        DrawRectangleRec(box(destination), panel);
        const float zoom =
            std::min(destination.width / source.width, destination.height / source.height);
        Camera2D camera{};
        camera.target = {source.x, source.y};
        camera.offset = {destination.x + (destination.width - source.width * zoom) / 2,
                         destination.y};
        camera.zoom = zoom;
        const Vector2 local{source.x + (input.x - camera.offset.x) / zoom,
                            source.y + (input.y - camera.offset.y) / zoom};
        BeginScissorMode(static_cast<int>(destination.x), static_cast<int>(destination.y),
                         static_cast<int>(destination.width), static_cast<int>(destination.height));
        BeginMode2D(camera);
        draw(local);
        EndMode2D();
        EndScissorMode();
    };
    draw_panel(
        {494, 112, 460, 410}, layout.image_panel,
        [&](Vector2 local) { sonar_panel(model, sonar_image, local, font); }, content_mouse);
    // Native timeline: scan rotation and echo propagation have independent clocks.
    const auto t = layout.transport;
    float y = t.y;
    text_at("Angle " + std::to_string(model.ping.angle_grad) + " grad / " +
                number(model.ping.angle_grad * 0.9, 1) + " deg",
            t.x, y + 4 * s, 17);
    const auto angle = static_cast<unsigned>(
        std::lround(slider({t.x + t.width * .48F, y, t.width * .50F, 25 * s},
                           model.ping.angle_grad / 399.0, content_mouse, cyan) *
                    399));
    if (angle != model.ping.angle_grad) {
        model.set_angle(angle);
    }
    y += 30 * s;
    if (button(font, model.scan_playing ? "Pause 360 scan" : "Play 360 scan", {t.x, y, 150 * s, h},
               content_mouse, model.scan_playing)) {
        model.set_scan_playing(!model.scan_playing);
    }
    if (button(font, "-", {t.x + 160 * s, y, 28 * s, h}, content_mouse)) {
        model.scan_speed_grad_s = std::max(2.0, model.scan_speed_grad_s - 10);
    }
    if (button(font, "+", {t.x + 196 * s, y, 28 * s, h}, content_mouse)) {
        model.scan_speed_grad_s = std::min(80.0, model.scan_speed_grad_s + 10);
    }
    text_at(number(400 / model.scan_speed_grad_s, 1) + " s/revolution (illustration, static pose)",
            t.x + 238 * s, y + 5 * s, 15, muted);
    y += 32 * s;
    if (button(font, model.playing ? "Pause pulse" : "Play pulse", {t.x, y, 110 * s, h},
               content_mouse)) {
        if (model.progress >= 1) {
            model.progress = 0;
        }
        model.playing = !model.playing;
    }
    if (button(font, "Replay", {t.x + 120 * s, y, 80 * s, h}, content_mouse)) {
        model.progress = 0;
        model.playing = true;
    }
    const double progress =
        slider({t.x + 218 * s, y, t.width - 230 * s, h}, model.progress, content_mouse, amber);
    if (progress != model.progress) {
        model.progress = progress;
        model.playing = false;
    }
    std::string timing = "Manual angle/image selection pauses the 360 scan.";
    if (model.wavefront_mode) {
        timing = "One-way front t=" + number(model.progress * model.wavefront.duration_s() * 1000) +
                 " ms; slowed to 5 s playback.";
    } else if (const auto *echo = model.echo()) {
        timing = "Echo: range " + number(echo->apparent_range_m) + " m / return " +
                 number(echo->travel_time_s * 1000) + " ms; " +
                 (model.progress <= .5 ? "outward" : "return on same route");
    }
    text_at(timing, t.x, t.y + t.height - 17 * s, 14, muted);
    if (model.profile_open) {
        // Profile geometry uses its whole native width; text and the range axis stay readable.
        const auto p = layout.profile;
        DrawRectangleRec(box(p), panel);
        text_at("ECHO PROFILE  |  direct / multipath / particles", p.x + 10 * s, p.y + 7 * s, 16,
                muted);
        const Rectangle plot{p.x + 38 * s, p.y + 33 * s, p.width - 54 * s, p.height - 80 * s};
        const float base = plot.y + plot.height;
        for (int level = 0; level <= 2; ++level) {
            const float py = plot.y + static_cast<float>(level) * plot.height / 2;
            DrawLineV({plot.x, py}, {plot.x + plot.width, py}, Color{48, 65, 82, 255});
            text_at(number(-model.dynamic_range_db * level / 2 - model.gain_db, 0), p.x + 3 * s,
                    py - 6, 13, muted);
        }
        for (const auto &bin : model.ping.profile) {
            const float px = plot.x + static_cast<float>(bin.center_range_m /
                                                         model.config.settings.max_range_m) *
                                          plot.width;
            const auto line = [&](double energy, Color color, float off) {
                DrawLineV({px + off, base},
                          {px + off,
                           base - static_cast<float>(model.display_level(energy)) * plot.height},
                          color);
            };
            if (model.channel == Channel::Combined || model.channel == Channel::Direct) {
                line(bin.direct_intensity, cyan, 0);
            }
            if (model.channel == Channel::Combined || model.channel == Channel::Multipath) {
                line(bin.multipath_intensity, amber, 1);
            }
            if (model.channel == Channel::Combined || model.channel == Channel::Volume) {
                line(bin.volume_intensity, green, 2);
            }
        }
        for (int tick = 0; tick <= 4; ++tick) {
            text_at(number(model.config.settings.max_range_m * tick / 4, 1),
                    plot.x + static_cast<float>(tick) * plot.width / 4 - 8, base + 3, 13, muted);
        }
        text_at("Apparent range (m); dB reference shared with scan image", p.x + 10 * s,
                p.y + p.height - 18, 13, muted);
        if (CheckCollisionPointRec(content_mouse, plot) &&
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            model.set_scan_playing(false);
            model.select_range((content_mouse.x - plot.x) / plot.width *
                               model.config.settings.max_range_m);
        }
    }
    if (drawer) {
        const Box p = model.settings_open ? layout.configuration : layout.inspector;
        DrawRectangle(static_cast<int>(p.x - 5), static_cast<int>(p.y - 5),
                      static_cast<int>(p.width + 10), static_cast<int>(p.height + 10),
                      Color{3, 9, 15, 235});
        const Vector2 drawer_mouse =
            model.help_open || model.camera_dragging ? Vector2{-10000, -10000} : mouse;
        if (model.settings_open) {
            draw_panel(
                {978, 457, 438, 406}, p, [&](Vector2 local) { configuration(model, local, font); },
                drawer_mouse);
        } else {
            draw_panel(
                {978, 78, 438, 365}, p, [&](Vector2 local) { inspector(model, local, font); },
                drawer_mouse);
        }
    }
    text_at(model.status, layout.footer.x, layout.footer.y + 3, 14, model.error ? amber : muted);
    text_at("Approximate geometry/scattering; no calibrated pressure, interference or "
            "measured-data classification.",
            layout.footer.x, layout.footer.y + 23 * s, 13, muted);
    if (model.help_open) {
        DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Color{0, 0, 0, 210});
        const float width = std::min(850.0F, static_cast<float>(GetScreenWidth()) - 40),
                    height = std::min(600.0F, static_cast<float>(GetScreenHeight()) - 36);
        const float left = (static_cast<float>(GetScreenWidth()) - width) / 2,
                    top = (static_cast<float>(GetScreenHeight()) - height) / 2;
        DrawRectangleRounded({left, top, width, height}, .03F, 4, panel);
        text_at("READING THE SONAR", left + 20 * s, top + 18 * s, 24);
        float py = top + 62 * s;
        const auto paragraph = [&](const char *heading, const char *body) {
            text_at(heading, left + 20 * s, py, 18, cyan);
            py += 27;
            std::istringstream words(body);
            std::string word, line;
            while (words >> word) {
                const auto candidate = line.empty() ? word : line + ' ' + word;
                if (!line.empty() &&
                    MeasureTextEx(font, candidate.c_str(), 18.0F, .5F).x > width - 40 * s) {
                    text_at(line, left + 20 * s, py, 17);
                    py += 24;
                    line = word;
                } else {
                    line = candidate;
                }
            }
            text_at(line, left + 20 * s, py, 17);
            py += 38;
        };
        paragraph("Why does multipath make a larger ghost rectangle?",
                  "The sonar shows half the full sound route. A wall 4 m away gives an 8 m "
                  "roundtrip and is shown at 4 m. A reflected 12 m roundtrip is shown at 6 m along "
                  "the head bearing, even if no wall is there.");
        paragraph("The circles are a ruler, not pool walls",
                  "Rings mark apparent range. Bright shapes come from returns. Direct, Multi and "
                  "Particles isolate modeled sources with one shared brightness reference.");
        paragraph("One narrow fan rotates through 360 degrees",
                  "Ping360 has a nominal 2 degree horizontal by 25 degree vertical beam. One ping "
                  "illuminates that fan, not the entire circle. Play 360 scan uses an illustration "
                  "speed with a stationary vehicle.");
        paragraph("Markers and fronts are viewing aids",
                  "Particle markers only hide background dots; particle echoes remain enabled. "
                  "Wavefront is geometric, not pressure or interference. Drag in 3D to orbit, "
                  "wheel to zoom, Reset to restore.");
        if (button(font, "Close help [H]",
                   {left + width / 2 - 80 * s, top + height - 40 * s, 160 * s, 28 * s}, mouse)) {
            model.help_open = false;
        }
    }
}

} // namespace viewer
