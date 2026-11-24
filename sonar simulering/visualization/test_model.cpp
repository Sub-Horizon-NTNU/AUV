#include "model.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {

void check(bool condition, const char *message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(sonar::Vec3 actual, sonar::Vec3 expected, const char *message) {
    check(sonar::length(actual - expected) < 1e-8, message);
}

} // namespace

int main() {
    try {
        viewer::Model model({});
        auto config = model.config;
        config.settings.horizontal_rays = 1;
        config.settings.vertical_rays = 1;
        config.pose.yaw_deg = 90;
        check(model.apply(config, 0), "rotated configuration should apply");
        model.select_range(7.5);
        const auto *echo = model.echo();
        check(echo && echo->is_multipath(), "profile selection should find a later echo");
        near(viewer::apparent_position(*echo), {5.0, 10.0, 1.5},
             "apparent position must follow the rotated pool-frame launch direction");
        near(viewer::path_position(*echo, 0.5), {5.0, 0.0, 1.5},
             "halfway marker should reach the backscatter surface");
        near(viewer::path_position(*echo, 1.0), config.pose.position_m,
             "complete marker should return to the sonar");
        const auto *cached_data = model.ping.echoes.data();
        model.tick(0.25);
        model.projection = viewer::Projection::Side;
        check(model.ping.echoes.data() == cached_data, "playback should reuse the cached ping");
        auto invalid = model.config;
        invalid.pose.position_m.z = -1;
        check(!model.apply(invalid, 0), "invalid draft should be rejected");
        check(model.ping.echoes.data() == cached_data && model.config.pose.position_m.z == 1.5,
              "rejected draft must preserve the previous valid ping and configuration");
        model.cycle_echo(-1);
        check(model.echo() != nullptr, "previous echo should remain selectable");
        model.change_angle(-1);
        check(model.ping.angle_grad == 399, "angle should wrap from 0 to 399");
        config = model.config;
        config.settings.max_range_m = 0.75;
        check(model.apply(config, 0) && model.echo() == nullptr,
              "empty results should be supported");
        model.cycle_echo(1);
        model.select_range(0.5);
        std::cout << "PASS: viewer cached state, pose-correct ghost, animation, selection, invalid "
                     "draft, empty results\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
