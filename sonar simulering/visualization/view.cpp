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
constexpr Rectangle world_area{24, 112, 450, 410};
constexpr Rectangle sonar_image_area{566, 172, 316, 316};
constexpr Rectangle chart_area{60, 700, 860, 125};

std::string number(double value, int precision = 2) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

void label(Font font, const std::string &value, float x, float y, int size = 18,
           Color color = text) {
    DrawTextEx(font, value.c_str(), {x, y}, static_cast<float>(size), 0.5F, color);
}

void wrapped(Font font, const std::string &value, float x, float y, float width, int size,
             Color color) {
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

void draw_world3d(const Model &model, RenderTexture2D target) {
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
    Camera3D camera{};
    camera.position = {center.x + std::cos(azimuth) * extent * 1.7F, center.y + extent * 1.05F,
                       center.z + std::sin(azimuth) * extent * 1.7F};
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
                       point(front.points[link.second].position_m), cyan);
        }
        for (const auto &sample : front.points) {
            DrawSphere(point(sample.position_m), marker_scale * 0.006F,
                       sample.segment_index == 0 ? cyan : amber);
        }
    }
    EndMode3D();
    EndTextureMode();
}

void draw_world2d(const Model &model, Font font) {
    BeginScissorMode(24, 152, 450, 285);
    const bool side = model.projection == Projection::Side;
    const auto &pool = model.config.pool;
    const double vertical_extent = side ? pool.depth_m : pool.width_m;
    // Include the apparent straight-line point, which can lie outside the real pool.
    double xmin = 0;
    double xmax = pool.length_m;
    double ymin = 0;
    double ymax = vertical_extent;
    if (const auto *echo = model.echo()) {
        const auto apparent = apparent_position(*echo);
        xmin = std::min(0.0, apparent.x);
        xmax = std::max(xmax, apparent.x);
        ymin = std::min(0.0, side ? apparent.z : apparent.y);
        ymax = std::max(ymax, side ? apparent.z : apparent.y);
    }
    const float scale = std::min((world_area.width - 100) / static_cast<float>(xmax - xmin),
                                 (world_area.height - 150) / static_cast<float>(ymax - ymin));
    const float left =
        world_area.x + (world_area.width - static_cast<float>(xmax - xmin) * scale) / 2;
    const float bottom =
        world_area.y + (world_area.height + static_cast<float>(ymax - ymin) * scale) / 2 - 15;
    const auto project = [&](sonar::Vec3 value) {
        return Vector2{left + static_cast<float>(value.x - xmin) * scale,
                       bottom - static_cast<float>((side ? value.z : value.y) - ymin) * scale};
    };
    const auto lower = project({0, 0, 0});
    const auto upper = project({pool.length_m, pool.width_m, pool.depth_m});
    DrawRectangleRec({lower.x, upper.y, upper.x - lower.x, lower.y - upper.y},
                     Color{27, 46, 61, 255});
    DrawRectangleLinesEx({lower.x, upper.y, upper.x - lower.x, lower.y - upper.y}, 2,
                         Color{104, 146, 171, 255});
    label(font, side ? "SIDE / x-z (height)" : "TOP / x-y", world_area.x + 18, world_area.y + 14,
          17, muted);
    label(font, "x", upper.x + 10, lower.y - 6, 18, muted);
    label(font, side ? "z" : "y", lower.x - 20, upper.y - 12, 18, muted);
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
                       project(front.points[link.second].position_m), 2, cyan);
        }
        for (const auto &sample : front.points) {
            DrawCircleV(project(sample.position_m), 3, sample.segment_index == 0 ? cyan : amber);
        }
    }
    DrawCircleV(project(model.config.pose.position_m), 6, cyan);
    EndScissorMode();
}

void profile(Model &model, Vector2 mouse, Font font) {
    DrawRectangleRounded({24, 637, 930, 226}, 0.04F, 4, panel);
    label(font, "RECEIVED ECHO PROFILE", 44, 653, 20);
    label(font, "cyan: direct   amber: multipath   green: particles", 410, 655, 17, muted);
    const auto height = [&](double energy) {
        return static_cast<float>(model.display_level(energy)) * chart_area.height;
    };
    for (int level = 0; level <= 3; ++level) {
        const float y = chart_area.y + static_cast<float>(level) * chart_area.height / 3;
        DrawLineV({chart_area.x, y}, {chart_area.x + chart_area.width, y}, Color{48, 65, 82, 255});
        label(font, number(-model.dynamic_range_db * level / 3 - model.gain_db, 0), 25, y - 7, 13,
              muted);
    }
    const auto &bins = model.ping.profile;
    for (std::size_t index = 0; index < bins.size(); ++index) {
        const float x = chart_area.x + static_cast<float>(bins[index].center_range_m /
                                                          model.config.settings.max_range_m) *
                                           chart_area.width;
        const float bottom = chart_area.y + chart_area.height;
        if (model.channel == Channel::Combined || model.channel == Channel::Direct) {
            DrawLineEx({x, bottom}, {x, bottom - height(bins[index].direct_intensity)}, 2, cyan);
        }
        if (model.channel == Channel::Combined || model.channel == Channel::Multipath) {
            DrawLineEx({x + 1, bottom}, {x + 1, bottom - height(bins[index].multipath_intensity)},
                       2, amber);
        }
        if (model.channel == Channel::Combined || model.channel == Channel::Volume) {
            DrawLineEx({x + 2, bottom}, {x + 2, bottom - height(bins[index].volume_intensity)}, 2,
                       green);
        }
    }
    const float blind_width =
        static_cast<float>(sonar::minimum_range_m / model.config.settings.max_range_m) *
        chart_area.width;
    DrawRectangleRec({chart_area.x, chart_area.y, blind_width, chart_area.height},
                     Color{103, 115, 128, 50});
    if (const auto *echo = model.echo()) {
        const float x = chart_area.x + static_cast<float>(echo->apparent_range_m /
                                                          model.config.settings.max_range_m) *
                                           chart_area.width;
        DrawLineV({x, chart_area.y - 5}, {x, chart_area.y + chart_area.height}, WHITE);
    }
    for (int tick = 0; tick <= 4; ++tick) {
        const float x = chart_area.x + static_cast<float>(tick) * chart_area.width / 4;
        label(font, number(model.config.settings.max_range_m * tick / 4.0, 1), x - 10, 830, 15,
              muted);
    }
    label(font, "Apparent range (m)  |  click a peak to inspect its nearest echo", 232, 841, 15,
          muted);
    label(font,
          "dB relative to entire scan peak (shared with image); gain " + number(model.gain_db, 0) +
              " dB",
          44, 680, 15, muted);
    if (CheckCollisionPointRec(mouse, chart_area) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        model.select_range((mouse.x - chart_area.x) / chart_area.width *
                           model.config.settings.max_range_m);
    }
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
        wrapped(font,
                "750 kHz default; wavelength " +
                    number(settings.sound_speed_m_s / settings.frequency_hz * 1000) + " mm. Seed " +
                    std::to_string(settings.volume_seed) +
                    ". Smooth walls return strongly near normal incidence; particles are "
                    "single-scatter proxies.",
                998, 749, 394, 15, muted);
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

void render_world(const Model &model, RenderTexture2D world) {
    draw_world3d(model, world);
}

void render_sonar(const Model &model, SonarTexture &image) {
    if (image.texture.id && image.revision == model.scan_revision &&
        image.channel == model.channel && image.gain_db == model.gain_db &&
        image.dynamic_range_db == model.dynamic_range_db) {
        return;
    }
    constexpr int size = 384;
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

void draw_interface(Model &model, RenderTexture2D world, Texture2D sonar_image, Vector2 mouse,
                    Font font) {
    ClearBackground(background);
    label(font, "PING360  /  FOLLOW THE SOUND", 24, 20, 28);
    label(font,
          "Compare ray geometry with a stationary 400-angle sonar image. Smooth walls, rough "
          "surfaces and particles change the return.",
          24, 55, 18, muted);
    label(font, "RAY PATH", 24, 88, 20);
    label(font, "SIMULATED SONAR IMAGE", 494, 88, 20);
    if (button(font, "3D", {190, 78, 54, 30}, mouse, model.projection == Projection::Perspective)) {
        model.projection = Projection::Perspective;
    }
    if (button(font, "Top", {250, 78, 54, 30}, mouse, model.projection == Projection::Top)) {
        model.projection = Projection::Top;
    }
    if (button(font, "Side", {310, 78, 54, 30}, mouse, model.projection == Projection::Side)) {
        model.projection = Projection::Side;
    }
    if (button(font, "Orbit", {370, 78, 104, 30}, mouse)) {
        model.orbit_deg += 25;
    }
    DrawRectangleRec(world_area, background);
    if (model.projection == Projection::Perspective) {
        DrawTexturePro(world.texture,
                       {0, 0, static_cast<float>(world.texture.width),
                        -static_cast<float>(world.texture.height)},
                       world_area, {0, 0}, 0, WHITE);
    } else {
        draw_world2d(model, font);
    }
    DrawRectangleLinesEx(world_area, 1, Color{58, 78, 98, 255});
    if (button(font, "Ray paths", {35, 122, 150, 28}, mouse, !model.wavefront_mode)) {
        model.wavefront_mode = false;
        model.progress = 0;
        model.playing = true;
    }
    if (button(font, "Wavefront", {195, 122, 150, 28}, mouse, model.wavefront_mode)) {
        model.wavefront_mode = true;
        model.progress = 0;
        model.playing = true;
    }
    label(font,
          model.wavefront_mode ? "Cyan: incident front   Amber: reflected front"
                               : "Cyan: sonar / pulse    Amber: reflected path",
          35, 466, 16, muted);
    label(font,
          model.wavefront_mode ? "Coarse geometric beamfront; not a pressure field"
                               : "Green: water particles",
          35, 487, 15, model.wavefront_mode ? muted : green);
    label(font,
          model.wavefront_mode ? "One-way propagation within the 2 x 25 degree beam"
                               : "Purple: apparent echo position (may be offscreen)",
          35, 445, 15, model.wavefront_mode ? muted : ghost);
    sonar_panel(model, sonar_image, mouse, font);

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
    label(font,
          "Ping angle  " + std::to_string(model.ping.angle_grad) + " grad  /  " +
              number(model.ping.angle_grad * 0.9, 1) + " deg    [Left / Right]",
          26, 539, 18);
    const double angle = slider({555, 532, 398, 28}, model.ping.angle_grad / 399.0, mouse, cyan);
    const auto angle_grad = static_cast<unsigned>(std::lround(angle * 399));
    if (angle_grad != model.ping.angle_grad) {
        model.set_angle(angle_grad);
    }
    if (button(font, model.playing ? "Pause" : "Play", {24, 576, 96, 32}, mouse)) {
        if (model.progress >= 1) {
            model.progress = 0;
        }
        model.playing = !model.playing;
    }
    if (button(font, "Replay", {130, 576, 96, 32}, mouse)) {
        model.progress = 0;
        model.playing = true;
    }
    const double scrubbed = slider({248, 578, 420, 28}, model.progress, mouse, amber);
    if (scrubbed != model.progress) {
        model.progress = scrubbed;
        model.playing = false;
    }
    label(font,
          model.wavefront_mode    ? "ONE-WAY FRONT"
          : model.progress <= 0.5 ? "OUTWARD"
                                  : "RETURN / same path",
          690, 584, 17, model.progress <= 0.5 ? cyan : amber);
    if (model.wavefront_mode) {
        label(font,
              "5 s playback | one-way window " + number(model.wavefront.duration_s() * 1000) +
                  " ms | t = " + number(model.progress * model.wavefront.duration_s() * 1000) +
                  " ms | Space: pause",
              25, 616, 15, muted);
    } else if (const auto *echo = model.echo()) {
        label(font,
              "5 s playback, slowed from " + number(echo->travel_time_s * 1000) +
                  " ms acoustic time  |  t = " +
                  number(echo->travel_time_s * model.progress * 1000) + " ms  |  Space: pause",
              25, 616, 15, muted);
    }
    profile(model, mouse, font);
    inspector(model, mouse, font);
    configuration(model, mouse, font);
    label(font, model.status, 24, 868, 16, model.error ? amber : muted);
    label(font,
          "MODEL LIMIT: reciprocal surface paths + sparse single-particle returns. No phase, "
          "diffraction or calibrated energy; no real-data classifier.",
          24, 889, 13, muted);
}

} // namespace viewer
