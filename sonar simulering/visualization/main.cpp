#include "model.hpp"
#include "view.hpp"

#include "raylib.h"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

// Texture objects must be released before their window's graphics context.
struct GraphicsResources {
    RenderTexture2D canvas{};
    RenderTexture2D world{};
    viewer::SonarTexture sonar_image;
    Font font{};
    bool owns_font{};

    GraphicsResources() = default;
    GraphicsResources(const GraphicsResources &) = delete;
    GraphicsResources &operator=(const GraphicsResources &) = delete;

    ~GraphicsResources() {
        if (sonar_image.texture.id != 0) {
            UnloadTexture(sonar_image.texture);
        }
        if (owns_font) {
            UnloadFont(font);
        }
        if (world.id != 0) {
            UnloadRenderTexture(world);
        }
        if (canvas.id != 0) {
            UnloadRenderTexture(canvas);
        }
        if (IsWindowReady()) {
            CloseWindow();
        }
    }
};

} // namespace

int main(int argc, char **argv) {
    try {
        std::filesystem::path config_path;
        std::filesystem::path screenshot_path;
        bool smoke_test = false;
        bool wavefront_mode = false;
        bool scattering_controls = false;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--smoke-test") {
                smoke_test = true;
            } else if (argument == "--wavefront") {
                wavefront_mode = true;
            } else if (argument == "--scattering") {
                scattering_controls = true;
            } else if (argument == "--help") {
                std::cout
                    << "sonar_viewer [--config FILE] [--wavefront] [--scattering] [--smoke-test] "
                       "[--screenshot FILE.png]\n"
                       "Interactive stationary Ping360 model. Illustrative defaults if no config.\n"
                       "Arrows: angle/echo, Space: pause, Escape: close.\n"
                       "Smoke test renders 8 frames then exits. Screenshot is written only when "
                       "requested.\n";
                return 0;
            } else if ((argument == "--config" || argument == "--screenshot") && index + 1 < argc) {
                (argument == "--config" ? config_path : screenshot_path) = argv[++index];
            } else {
                throw std::invalid_argument("unknown option or missing value: " + argument);
            }
        }
        viewer::Model model(config_path);
        model.wavefront_mode = wavefront_mode;
        model.scattering_controls = scattering_controls;
        if (smoke_test && wavefront_mode) {
            model.progress = 0.43;
            model.playing = false;
        }
        GraphicsResources graphics;
        SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
        InitWindow(1280, 800, "Ping360 - Follow the sound");
        if (!IsWindowReady()) {
            throw std::runtime_error("raylib could not create a graphics window");
        }
        SetWindowMinSize(1000, 625);
        SetTargetFPS(60);
        // Use a locally installed UI font; other platforms retain raylib's default font.
        graphics.font = GetFontDefault();
        const char *font_path = "C:/Windows/Fonts/segoeui.ttf";
        if (FileExists(font_path)) {
            const Font loaded = LoadFontEx(font_path, 32, nullptr, 0);
            if (loaded.texture.id != 0) {
                graphics.font = loaded;
                graphics.owns_font = loaded.texture.id != GetFontDefault().texture.id;
                SetTextureFilter(graphics.font.texture, TEXTURE_FILTER_BILINEAR);
            }
        }
        graphics.canvas = LoadRenderTexture(viewer::canvas_width, viewer::canvas_height);
        graphics.world = LoadRenderTexture(450, 410);
        SetTextureFilter(graphics.canvas.texture, TEXTURE_FILTER_BILINEAR);
        SetTextureFilter(graphics.world.texture, TEXTURE_FILTER_BILINEAR);
        const auto &canvas = graphics.canvas;
        const auto &world = graphics.world;
        unsigned frames = 0;
        while (!WindowShouldClose()) {
            const float scale =
                std::min(static_cast<float>(GetScreenWidth()) / viewer::canvas_width,
                         static_cast<float>(GetScreenHeight()) / viewer::canvas_height);
            const Vector2 offset{
                (static_cast<float>(GetScreenWidth()) - viewer::canvas_width * scale) / 2,
                (static_cast<float>(GetScreenHeight()) - viewer::canvas_height * scale) / 2};
            const Vector2 window_mouse = GetMousePosition();
            const Vector2 mouse{(window_mouse.x - offset.x) / scale,
                                (window_mouse.y - offset.y) / scale};
            model.tick(GetFrameTime());
            viewer::render_world(model, world);
            viewer::render_sonar(model, graphics.sonar_image);
            BeginTextureMode(canvas);
            viewer::draw_interface(model, world, graphics.sonar_image.texture, mouse,
                                   graphics.font);
            EndTextureMode();
            BeginDrawing();
            ClearBackground(BLACK);
            DrawTexturePro(
                canvas.texture,
                {0, 0, static_cast<float>(viewer::canvas_width),
                 -static_cast<float>(viewer::canvas_height)},
                {offset.x, offset.y, viewer::canvas_width * scale, viewer::canvas_height * scale},
                {0, 0}, 0, WHITE);
            EndDrawing();
            ++frames;
            if (smoke_test && frames >= 8) {
                if (!screenshot_path.empty()) {
                    TakeScreenshot(screenshot_path.string().c_str());
                }
                break;
            }
        }
        if (!smoke_test && !screenshot_path.empty()) {
            TakeScreenshot(screenshot_path.string().c_str());
        }
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "sonar_viewer: " << error.what() << '\n';
        return 1;
    }
}
