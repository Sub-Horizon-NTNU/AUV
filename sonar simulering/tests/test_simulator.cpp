#include "sonar/io.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void check(bool condition, const std::string &message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const std::string &message, double tolerance = 1e-10) {
    if (!std::isfinite(actual) || !std::isfinite(expected) ||
        std::abs(actual - expected) > tolerance * std::max(1.0, std::abs(expected))) {
        throw std::runtime_error(message + ": got " + std::to_string(actual) + ", expected " +
                                 std::to_string(expected));
    }
}

template <class Action> void rejects(Action action, const std::string &message) {
    try {
        action();
    } catch (const std::invalid_argument &) {
        return;
    }
    throw std::runtime_error(message + ": invalid input was accepted");
}

sonar::Config single_ray() {
    sonar::Config config;
    config.settings.horizontal_rays = 1;
    config.settings.vertical_rays = 1;
    return config;
}

double profile_energy(const sonar::Ping &ping, bool multipath) {
    double sum = 0.0;
    for (const auto &bin : ping.profile) {
        sum += multipath ? bin.multipath_intensity : bin.direct_intensity;
    }
    return sum;
}

void normal_wall_and_multipath() {
    const auto config = single_ray();
    const auto ping = sonar::simulate_ping(config, 0);
    check(ping.echoes.size() == 2, "20 m range should contain normal returns at 5 m and 15 m");
    const auto &direct = ping.echoes[0];
    near(direct.apparent_range_m, 5.0, "wall distance");
    near(direct.total_path_m, 10.0, "direct roundtrip length");
    near(direct.travel_time_s, 10.0 / 1480.0, "direct roundtrip time");
    check(!direct.is_multipath() && direct.interaction_count == 1, "direct classification");
    const auto &multi = ping.echoes[1];
    near(multi.apparent_range_m, 15.0, "second apparent range");
    near(multi.total_path_m, 30.0, "multipath roundtrip length");
    check(multi.is_multipath() && multi.interaction_count == 3, "multipath classification");
    check(multi.path.size() == 5, "multipath includes outward and return contacts");
    near(multi.path[1].position_m.x, 10.0, "outward xmax contact");
    near(multi.path[2].position_m.x, 0.0, "backscatter xmin contact");
    near(multi.path[3].position_m.x, 10.0, "return xmax contact");
    near(profile_energy(ping, false), direct.relative_intensity, "direct pulse energy");
    near(profile_energy(ping, true), multi.relative_intensity, "multipath pulse energy");

    const double expected_direct = 0.8 * 0.1 * std::pow(1.0 / 5.0, 4) * std::exp(-0.01 * 10.0);
    near(direct.relative_intensity, expected_direct, "boundary and transmission energy");
    const double expected_multi =
        std::pow(0.8 * 0.9, 2) * 0.8 * 0.1 * std::pow(1.0 / 15.0, 4) * std::exp(-0.01 * 30.0);
    near(multi.relative_intensity, expected_multi, "two-way specular survival energy");
}

void oblique_reflection() {
    const double component = std::sqrt(0.5);
    const auto reflected = sonar::reflect({component, component, 0.0}, {1.0, 0.0, 0.0});
    near(reflected.x, -component, "reflection reverses normal component");
    near(reflected.y, component, "reflection preserves tangential component");
    near(sonar::length(reflected), 1.0, "reflection preserves unit length");

    auto config = single_ray();
    config.pool.width_m = 20.0;
    config.pose.position_m.y = 10.0;
    config.settings.max_range_m = 50.0;
    const auto ping = sonar::simulate_ping(config, 50);
    check(ping.echoes.size() >= 2, "oblique case must include multipath");
    near(ping.echoes[0].apparent_range_m, 5.0 * std::sqrt(2.0), "oblique xmax distance");
    near(ping.echoes[1].path[2].position_m.x, 5.0, "second hit reflected x");
    near(ping.echoes[1].path[2].position_m.y, 20.0, "second hit ymax");
}

void range_and_blind_zone() {
    auto config = single_ray();
    config.settings.max_range_m = 6.0;
    const auto limited = sonar::simulate_ping(config, 0);
    check(limited.echoes.size() == 1, "range cutoff uses accumulated outward length");
    config.settings.max_range_m = 20.0;
    config.pose.position_m.x = 9.5;
    const auto near_wall = sonar::simulate_ping(config, 0);
    check(!near_wall.echoes[0].in_receive_window, "sub-0.75 m return is metadata only");
    near(profile_energy(near_wall, false), 0.0, "blind-zone direct profile suppression");
    check(profile_energy(near_wall, true) > 0.0, "later in-window multipath remains visible");

    config = single_ray();
    config.settings.max_range_m = 5.001;
    const auto clipped = sonar::simulate_ping(config, 0);
    const double pulse_range_m = 1480.0 * 20e-6 / 2.0;
    const double expected = clipped.echoes[0].relative_intensity * 0.001 / pulse_range_m;
    near(profile_energy(clipped, false), expected, "partial final-bin pulse is retained", 1e-9);
    config.settings.max_range_m = 5.0;
    const auto at_end = sonar::simulate_ping(config, 0);
    check(!at_end.echoes[0].in_receive_window, "receive interval excludes right endpoint");
    near(profile_energy(at_end, false), 0.0, "pulse starting at range limit has no in-window tail");
}

void vertical_fan_and_orientation() {
    auto config = single_ray();
    config.pool.length_m = 100.0;
    config.pool.width_m = 100.0;
    config.pose.position_m = {50.0, 50.0, 1.5};
    config.settings.vertical_rays = 3;
    config.settings.max_outward_hits = 1;
    config.settings.max_range_m = 49.0;
    const auto fan = sonar::simulate_ping(config, 0);
    check(fan.echoes.size() == 2, "vertical fan reaches floor and surface before distant walls");
    check(fan.echoes[0].path[1].surface_mask == (1U << 4), "lower fan hits floor");
    check(fan.echoes[1].path[1].surface_mask == (1U << 5), "upper fan hits water surface");

    config = single_ray();
    config.pose.pitch_deg = 90.0;
    const auto downward = sonar::simulate_ping(config, 0);
    near(downward.echoes[0].apparent_range_m, 1.5, "positive right-handed pitch points down");
    check(downward.echoes[0].path[1].surface_mask == (1U << 4), "pitch rotation reaches floor");
    config.pose.pitch_deg = 0.0;
    config.pose.yaw_deg = 90.0;
    near(sonar::simulate_ping(config, 0).echoes[0].apparent_range_m, 2.5, "yaw reaches ymax");
}

void corner_and_closed_paths() {
    auto config = single_ray();
    config.pool.width_m = 10.0;
    config.pool.depth_m = 10.0;
    config.pose.position_m = {5.0, 5.0, 5.0};
    config.settings.max_range_m = 50.0;
    const auto corner = sonar::simulate_ping(config, 50);
    check(corner.echoes.size() == 4, "corner ray does not stall at zero-length contacts");
    check(corner.echoes[0].path[1].surface_mask == ((1U << 1) | (1U << 3)),
          "corner records both xmax and ymax");
    near(corner.echoes[1].path[2].position_m.x, 0.0, "corner continuation reflects x");
    near(corner.echoes[1].path[2].position_m.y, 0.0, "corner continuation reflects y");

    config = sonar::Config{};
    for (unsigned angle = 0; angle < 400; ++angle) {
        const auto ping = sonar::simulate_ping(config, angle);
        for (const auto &echo : ping.echoes) {
            double polyline_length = 0.0;
            for (std::size_t index = 1; index < echo.path.size(); ++index) {
                polyline_length +=
                    sonar::length(echo.path[index].position_m - echo.path[index - 1].position_m);
            }
            near(polyline_length, echo.total_path_m, "CSV-ready path is the complete roundtrip");
            near(sonar::length(echo.path.front().position_m - echo.path.back().position_m), 0.0,
                 "return reaches sonar");
            check(std::isfinite(echo.relative_intensity) && echo.relative_intensity >= 0.0,
                  "relative energy remains finite and nonnegative");
            check(echo.apparent_range_m <= config.settings.max_range_m,
                  "all reported arrivals fit the maximum total time");
        }
    }
    const auto first = sonar::simulate_ping(config, 37);
    const auto repeat = sonar::simulate_ping(config, 37);
    check(first.echoes.size() == repeat.echoes.size(), "repeat produces same echo count");
    for (std::size_t index = 0; index < first.profile.size(); ++index) {
        check(first.profile[index].direct_intensity == repeat.profile[index].direct_intensity &&
                  first.profile[index].multipath_intensity ==
                      repeat.profile[index].multipath_intensity,
              "repeat profile is deterministic");
    }
}

void coefficient_boundaries() {
    auto config = single_ray();
    config.settings.backscatter_fraction = 1.0;
    const auto all_backscatter = sonar::simulate_ping(config, 0);
    check(all_backscatter.echoes.size() == 1,
          "all-backscatter boundary has no specular continuation");
    config.settings.backscatter_fraction = 0.0;
    const auto no_backscatter = sonar::simulate_ping(config, 0);
    near(profile_energy(no_backscatter, false) + profile_energy(no_backscatter, true), 0.0,
         "zero backscatter has no modeled received energy");
    config = single_ray();
    config.pool.wall_reflection = 0.0;
    const auto absorbing = sonar::simulate_ping(config, 0);
    check(absorbing.echoes.size() == 1, "absorbing wall stops continuation");
    near(absorbing.echoes[0].relative_intensity, 0.0, "absorbing wall has no reflection");
}

void invalid_config_and_parser() {
    auto invalid = sonar::Config{};
    invalid.pool.length_m = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { sonar::simulate_ping(invalid, 0); }, "NaN pool dimension");
    invalid = sonar::Config{};
    invalid.settings.sound_speed_m_s = std::numeric_limits<double>::infinity();
    rejects([&] { sonar::validate(invalid); }, "infinite speed");
    invalid = sonar::Config{};
    invalid.pose.position_m.x = 0.0;
    rejects([&] { sonar::validate(invalid); }, "sonar on boundary");
    invalid = sonar::Config{};
    invalid.settings.range_samples = 1201;
    rejects([&] { sonar::validate(invalid); }, "sample count cap");
    invalid = sonar::Config{};
    invalid.settings.horizontal_rays = 101;
    invalid.settings.vertical_rays = 101;
    rejects([&] { sonar::validate(invalid); }, "ray product cap");
    invalid = sonar::Config{};
    invalid.settings.absorption_per_m = -0.01;
    rejects([&] { sonar::validate(invalid); }, "negative absorption");
    rejects([] { sonar::simulate_ping(sonar::Config{}, 400); }, "out-of-range head angle");

    std::istringstream valid("# toy\n pool_length_m = 12 # m\nsonar_x_m=6\nvertical_rays=5\n");
    const auto loaded = sonar::read_config(valid);
    near(loaded.pool.length_m, 12.0, "config dimension");
    check(loaded.settings.vertical_rays == 5, "config count");
    for (const auto &text :
         {"unknown=1", "pool_length_m=10\npool_length_m=12", "horizontal_rays=2.5", "sonar_x_m=nan",
          "max_range_m=20m", "sonar_y_m=", "no_equals", "horizontal_rays=-1"}) {
        rejects(
            [&] {
                std::istringstream input(text);
                sonar::read_config(input);
            },
            std::string("config rejection: ") + text);
    }
}

} // namespace

int main() {
    const std::vector<std::pair<const char *, std::function<void()>>> tests{
        {"normal wall and multipath", normal_wall_and_multipath},
        {"oblique reflection", oblique_reflection},
        {"range and blind zone", range_and_blind_zone},
        {"vertical fan and orientation", vertical_fan_and_orientation},
        {"corner and closed paths", corner_and_closed_paths},
        {"coefficient boundaries", coefficient_boundaries},
        {"invalid config and parser", invalid_config_and_parser},
    };
    unsigned failures = 0;
    for (const auto &[name, test] : tests) {
        try {
            test();
            std::cout << "PASS: " << name << '\n';
        } catch (const std::exception &error) {
            ++failures;
            std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        }
    }
    return failures == 0 ? 0 : 1;
}
