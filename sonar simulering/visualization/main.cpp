#include "model.hpp"
#include "view.hpp"

#include "raylib.h"

#include <algorithm>
#include <charconv>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

// Texture objects must be released before their window's graphics context.
struct GraphicsResources {
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
        int window_width = 1280;
        int window_height = 800;
        bool guide = false;
        bool scan = false;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--smoke-test") {
                smoke_test = true;
            } else if (argument == "--wavefront") {
                wavefront_mode = true;
            } else if (argument == "--scattering") {
                scattering_controls = true;
            } else if (argument == "--guide") {
                guide = true;
            } else if (argument == "--scan360") {
                scan = true;
            } else if ((argument == "--width" || argument == "--height") && index + 1 < argc) {
                const std::string value = argv[++index];
                int dimension{};
                const auto parsed =
                    std::from_chars(value.data(), value.data() + value.size(), dimension);
                const int lower = argument == "--width" ? 1000 : 700;
                if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
                    dimension < lower || dimension > 4000) {
                    throw std::invalid_argument(argument + " must be an integer from " +
                                                std::to_string(lower) + " to 4000");
                }
                (argument == "--width" ? window_width : window_height) = dimension;
            } else if (argument == "--help") {
                std::cout
                    << "sonar_viewer [--config FILE] [--wavefront] [--scattering] [--smoke-test] "
                       "[--screenshot FILE.png] [--width N --height N] [--guide] [--scan360]\n"
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
        model.settings_open = scattering_controls;
        model.help_open = guide;
        model.set_scan_playing(scan);
        if (smoke_test && wavefront_mode) {
            model.progress = 0.43;
            model.playing = false;
        }
        GraphicsResources graphics;
        SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
        InitWindow(window_width, window_height, "Ping360 - Follow the sound");
        if (!IsWindowReady()) {
            throw std::runtime_error("raylib could not create a graphics window");
        }
        SetWindowMinSize(1000, 700);
        SetExitKey(KEY_NULL);
        SetTargetFPS(60);
        // Use a locally installed UI font; other platforms retain raylib's default font.
        graphics.font = GetFontDefault();
        const char *font_path = "C:/Windows/Fonts/segoeui.ttf";
        if (FileExists(font_path)) {
            const Font loaded = LoadFontEx(font_path, 64, nullptr, 0);
            if (loaded.texture.id != 0) {
                graphics.font = loaded;
                graphics.owns_font = loaded.texture.id != GetFontDefault().texture.id;
                SetTextureFilter(graphics.font.texture, TEXTURE_FILTER_BILINEAR);
            }
        }
        viewer::CameraInput camera_input;
        unsigned frames = 0;
        while (!WindowShouldClose()) {
            if (IsKeyPressed(KEY_ESCAPE)) {
                if (model.help_open) {
                    model.help_open = false;
                } else if (model.settings_open || model.details_open) {
                    model.settings_open = false;
                    model.details_open = false;
                } else {
                    break;
                }
            }
            const auto layout =
                viewer::calculate_layout(GetScreenWidth(), GetScreenHeight(), model.profile_open);
            const int target_width = std::max(1, static_cast<int>(layout.world.width));
            const int target_height = std::max(1, static_cast<int>(layout.world.height));
            if (graphics.world.texture.width != target_width ||
                graphics.world.texture.height != target_height) {
                if (graphics.world.id) {
                    UnloadRenderTexture(graphics.world);
                }
                graphics.world = LoadRenderTexture(target_width, target_height);
                SetTextureFilter(graphics.world.texture, TEXTURE_FILTER_BILINEAR);
            }
            const Vector2 mouse = GetMousePosition();
            viewer::update_camera_input(model, layout, camera_input, mouse);
            model.tick(GetFrameTime());
            viewer::render_world(model, graphics.world);
            const float pixel_ratio =
                static_cast<float>(GetRenderWidth()) / static_cast<float>(GetScreenWidth());
            viewer::render_sonar(model, graphics.sonar_image,
                                 static_cast<int>(layout.image.width * pixel_ratio));
            BeginDrawing();
            viewer::draw_interface(model, graphics.world, graphics.sonar_image.texture, mouse,
                                   graphics.font, layout);
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
