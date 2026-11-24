#include "wavefront.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void check(bool condition, const char *message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const char *message) {
    check(std::abs(actual - expected) < 1e-9, message);
}

template <class Action> void rejects(Action action, const char *message) {
    try {
        action();
    } catch (const std::invalid_argument &) {
        return;
    }
    throw std::runtime_error(message);
}

} // namespace

int main() {
    try {
        sonar::Config config;
        const auto wave = viewer::make_wavefront(config, 0, 1, 1);
        check(wave.paths.size() == 1 && wave.paths.front().outward.size() == 3,
              "one ray retains only its two outward contacts, excluding echo retraces");
        near(wave.duration_s(), 20.0 / 1480.0, "time window is one-way Rmax/c");
        const auto initial = wave.sample_at_time(0);
        near(initial.points.front().position_m.x, 5, "front starts at transmitter");
        const auto incident = wave.sample_at_time(2.0 / 1480.0);
        check(incident.points.size() == 1 && incident.points[0].segment_index == 0,
              "incident front uses the first forward leg");
        near(incident.points[0].position_m.x, 7, "incident traveled distance equals c*t");
        const auto reflected = wave.sample_at_time(7.0 / 1480.0);
        check(reflected.points.size() == 1 && reflected.points[0].segment_index == 1,
              "front continues along the reflected outward leg");
        near(reflected.points[0].position_m.x, 8, "reflected front travels 2 m back from xmax");
        check(wave.sample_at_time(16.0 / 1480.0).points.empty(),
              "front disappears beyond the final known contact instead of following echo retraces");
        check(wave.sample_at_time(wave.duration_s() + 0.01).points.empty(),
              "front disappears beyond the one-way range window");
        rejects([&] { wave.sample_at_time(-0.01); }, "negative time must be rejected");
        rejects([&] { wave.sample_at_time(std::numeric_limits<double>::quiet_NaN()); },
                "non-finite time must be rejected");
        rejects([&] { viewer::make_wavefront(config, 0, 101, 101); },
                "oversized supplementary grid must be rejected by core validation");

        const auto grid = viewer::make_wavefront(config, 0, 3, 3);
        const auto before_wall = grid.sample_at_time(1.0 / 1480.0);
        check(before_wall.points.size() == 9 && before_wall.links.size() == 12,
              "regular fan has horizontal and vertical neighbor topology");
        for (const auto &sample : before_wall.points) {
            near(sonar::length(sample.position_m - config.pose.position_m), 1,
                 "geometric fan expands at equal traveled distance");
        }
        config.pool.width_m = 10;
        config.pose.position_m.y = 5;
        config.settings.max_range_m = 30;
        const auto corner = viewer::make_wavefront(config, 50, 3, 1);
        const auto split = corner.sample_at_time(7.1 / 1480.0);
        check(split.points.size() == 3 && split.links.empty(),
              "neighbors across differing reflection faces must not be stitched");
        config.settings.volume_scatterers = 4096;
        const auto with_particles = viewer::make_wavefront(config, 50, 3, 1);
        check(with_particles.paths.size() == corner.paths.size(),
              "particle proxies do not create additional incident-front branches");
        for (const auto &path : with_particles.paths) {
            for (const auto &vertex : path.outward) {
                check(!vertex.volume_scatter,
                      "only surface polylines define geometric propagation");
            }
        }
        std::cout << "PASS: geometric fan travel, reflection, topology, truncation, one-way clock "
                     "and input bounds\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
