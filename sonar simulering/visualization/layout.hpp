#pragma once

#include <algorithm>

namespace viewer {

struct Box {
    float x{};
    float y{};
    float width{};
    float height{};
};

struct Layout {
    float scale{};
    Box header;
    Box ray_panel;
    Box world;
    Box image_panel;
    Box image;
    Box transport;
    Box profile;
    Box inspector;
    Box configuration;
    Box footer;
};

// Native pixel rectangles, recomputed on resize. The sonar image remains square;
// the ray viewport changes aspect and has a matching native render target.
inline Layout calculate_layout(int width, int height, bool profile_open = false) {
    const float w = static_cast<float>(width);
    const float h = static_cast<float>(height);
    const float scale = std::clamp(std::min(w / 1440.0F, h / 900.0F), 0.78F, 1.45F);
    const float margin = 16 * scale;
    const float gap = 12 * scale;
    const float header_h = 72 * scale;
    const float footer_h = 54 * scale;
    const float right_w = std::clamp(w * 0.34F, 400.0F, 500.0F);
    const float left_w = w - 2 * margin;
    const float available_h = h - header_h - footer_h - 2 * margin;
    const float transport_h = 118 * scale;
    const float profile_h = profile_open ? std::clamp(available_h * 0.24F, 140.0F, 250.0F) : 0;
    const float visual_h = available_h - transport_h - profile_h - (profile_open ? 2 : 1) * gap;
    const float half_w = (left_w - gap) / 2;
    const float top = header_h;
    const Box ray{margin, top, half_w, visual_h};
    const Box image_panel{margin + half_w + gap, top, half_w, visual_h};
    const float image_size = std::max(40.0F, std::min(half_w - 32 * scale, visual_h - 148 * scale));
    const Box image{image_panel.x + (half_w - image_size) / 2, image_panel.y + 68 * scale,
                    image_size, image_size};
    const float right_x = w - margin - right_w;
    const float drawer_h = std::min(available_h, 490.0F);
    return {scale,
            {margin, margin, w - 2 * margin, header_h - margin},
            ray,
            {ray.x + 1, ray.y + 71 * scale, ray.width - 2, ray.height - 104 * scale},
            image_panel,
            image,
            {margin, top + visual_h + gap, left_w, transport_h},
            {margin, top + visual_h + gap + transport_h + gap, left_w, profile_h},
            {right_x, top, right_w, drawer_h},
            {right_x, top, right_w, drawer_h},
            {margin, h - footer_h, w - 2 * margin, footer_h}};
}

} // namespace viewer
