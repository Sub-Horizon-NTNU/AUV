#include "model.hpp"

#include <algorithm>
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
        const auto scan_revision = model.scan_revision;
        const auto *cached_scan = model.scan[0].data();
        const double reference = model.scan_reference;
        double observed_peak = 0;
        for (const auto &profile : model.scan) {
            check(profile.size() == config.settings.range_samples,
                  "all 400 angle profiles are cached");
            for (const auto &bin : profile) {
                observed_peak =
                    std::max(observed_peak,
                             bin.direct_intensity + bin.multipath_intensity + bin.volume_intensity);
            }
        }
        check(reference == observed_peak && reference > 0,
              "global reference uses strongest combined scan bin");
        model.set_angle(100);
        model.gain_db = 5;
        model.tick(0.5);
        check(model.scan_revision == scan_revision && model.scan[0].data() == cached_scan &&
                  model.scan_reference == reference,
              "angle, gain and playback do not rebuild or renormalize the scan");
        model.gain_db = 0;
        check(std::abs(model.display_level(reference * 1e-4) - 0.5) < 1e-10,
              "display uses energy decibels and the shared 80 dB span");
        check(model.select_image(0.375, 0) && model.ping.angle_grad == 0 && model.echo() &&
                  model.echo()->is_multipath(),
              "image click selects matching angle and nearest arrival");
        check(model.select_image(0, 0.2) && model.ping.angle_grad == 100,
              "sonar-local positive y maps to 100 grad regardless of world yaw");
        check(!model.select_image(2, 0), "image clicks outside the circle are ignored");
        model.set_channel(viewer::Channel::Volume);
        model.select_range(5);
        check(model.echo() == nullptr, "a channel with no arrivals must not select a surface echo");
        model.set_channel(viewer::Channel::Combined);
        check(model.echo() && model.accepts(*model.echo()),
              "switching from empty Particles to All must restore an eligible selection");
        model.set_channel(viewer::Channel::Volume);
        check(model.echo() == nullptr, "Particles remains empty when configured count is zero");
        model.set_channel(viewer::Channel::Direct);
        check(model.echo() && model.accepts(*model.echo()) &&
                  model.echo()->kind == sonar::EchoKind::Surface && !model.echo()->is_multipath(),
              "switching from empty Particles to Direct must restore a direct surface selection");
        check(model.scan_revision == scan_revision && model.scan[0].data() == cached_scan,
              "channel recovery must reuse the existing scan cache");
        model.set_channel(viewer::Channel::Combined);
        config = model.config;
        config.settings.max_range_m = 0.75;
        check(model.apply(config, 0) && model.echo() == nullptr,
              "empty results should be supported");
        model.cycle_echo(1);
        model.select_range(0.5);
        check(model.scan_reference == 0 && model.display_level(1) == 0,
              "empty scan normalization stays finite and dark");
        std::cout << "PASS: viewer cached state, pose-correct ghost, animation, selection, invalid "
                     "draft, empty results\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
