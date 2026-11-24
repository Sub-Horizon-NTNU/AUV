#include "wavefront.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

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

void inside(const viewer::WavefrontFrame &frame, const sonar::Config &config) {
    for (const auto &sample : frame.points) {
        const auto p = sample.position_m;
        check(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.x >= 0 &&
                  p.x <= config.pool.length_m && p.y >= 0 && p.y <= config.pool.width_m &&
                  p.z >= 0 && p.z <= config.pool.depth_m,
              "all ray and wall-seam points must remain finite and inside the pool");
    }
    for (const auto &link : frame.links) {
        check(link.first < frame.points.size() && link.second < frame.points.size(),
              "every topology index must identify an existing vertex");
    }
}

void normal_travel_and_clock() {
    sonar::Config config;
    const auto wave = viewer::make_wavefront(config, 0, 1, 1);
    check(wave.rays.size() == 1, "one ray should be cached independently of echo contacts");
    near(wave.duration_s(), 20.0 / 1480.0, "time window is one-way Rmax/c");
    const auto initial = wave.sample_at_time(0);
    near(initial.points.front().position_m.x, 5, "front starts at transmitter");
    const auto incident = wave.sample_at_time(2.0 / 1480.0);
    check(incident.points.size() == 1 && incident.points[0].segment_index == 0,
          "incident front uses the first forward leg");
    near(incident.points[0].position_m.x, 7, "incident traveled distance equals c*t");
    near(incident.points[0].direction.x, 1, "incident direction points toward xmax");
    near(sonar::length(incident.points[0].direction), 1, "direction is a unit vector");
    const auto reflected = wave.sample_at_time(7.0 / 1480.0);
    check(reflected.points.size() == 1 && reflected.points[0].segment_index == 1,
          "front continues along the reflected forward leg");
    near(reflected.points[0].position_m.x, 8, "reflected front travels 2 m back from xmax");
    near(reflected.points[0].direction.x, -1, "reflected vector reverses its normal component");
    const auto contact = wave.sample_at_time(5.0 / 1480.0);
    near(contact.points[0].direction.x, -1, "exact xmax contact uses outgoing direction");
    const auto late = wave.sample_at_time(16.0 / 1480.0);
    check(late.points.size() == 1, "front must persist past the final retained echo contact");
    near(late.points[0].position_m.x, 1, "front continues after its second physical reflection");
    const auto end = wave.sample_at_time(wave.duration_s());
    check(end.points.size() == 1, "range endpoint must retain every geometric ray");
    near(end.points[0].position_m.x, 5, "20 m propagation returns to the center");
    check(wave.sample_at_time(wave.duration_s() + 0.01).points.empty(),
          "front disappears only beyond the one-way range window");
    const auto before = wave.sample_at_time((5.0 - 1e-6) / 1480.0);
    const auto after = wave.sample_at_time((5.0 + 1e-6) / 1480.0);
    check(sonar::length(before.points[0].position_m - after.points[0].position_m) < 2e-6,
          "crossing a reflection time must not jump the front");
    const auto negative = viewer::make_wavefront(config, 200, 1, 1);
    const auto negative_contact = negative.sample_at_time(5.0 / 1480.0);
    near(negative_contact.points[0].direction.x, 1, "exact xmin contact uses outgoing direction");
    check(negative_contact.points[0].segment_index == 1,
          "exact contact with a minimum plane must count as reflected, like a maximum plane");
    rejects([&] { wave.sample_at_time(-0.01); }, "negative time must be rejected");
    rejects([&] { wave.sample_at_time(std::numeric_limits<double>::quiet_NaN()); },
            "non-finite time must be rejected");
    rejects([&] { viewer::make_wavefront(config, 400, 1, 1); }, "invalid angle must be rejected");
    rejects([&] { viewer::make_wavefront(config, 0, 0, 1); }, "zero resolution must be rejected");
    rejects([&] { viewer::make_wavefront(config, 0, 513, 513); },
            "oversized supplementary grid must be rejected");
    check(viewer::make_wavefront(config, 0, 101, 101).rays.size() == 10201,
          "geometric front resolution is independent of the core's 1024-ray quadrature cap");
}

void topology_and_wall_seams() {
    sonar::Config config;
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
    const auto next = corner.sample_at_time(7.10001 / 1480.0);
    for (std::size_t ray = 0; ray < corner.rays.size(); ++ray) {
        near(sonar::length(split.points[ray].direction), 1,
             "oblique reflected propagation vectors remain unit length");
        const auto observed = (next.points[ray].position_m - split.points[ray].position_m) * 1e5;
        near(sonar::length(observed - split.points[ray].direction), 0,
             "direction arrows must agree with observed front motion after a corner");
    }
    inside(split, config);
    check(split.points.size() > corner.rays.size(),
          "neighbors spanning reflections must acquire wall-seam vertices");
    for (std::size_t i = corner.rays.size(); i < split.points.size(); ++i) {
        const auto &point = split.points[i];
        check(point.ray_id == std::numeric_limits<unsigned>::max(),
              "interpolated seam vertices are distinct from physical ray samples");
        const auto p = point.position_m;
        check(std::min({p.x, config.pool.length_m - p.x, p.y, config.pool.width_m - p.y, p.z,
                        config.pool.depth_m - p.z}) < 1e-9,
              "each additional seam vertex must touch a physical pool plane");
    }
    std::vector<bool> connected(split.points.size(), false);
    connected[0] = true;
    for (std::size_t pass = 0; pass < split.points.size(); ++pass) {
        for (const auto &link : split.links) {
            if (connected[link.first] || connected[link.second]) {
                connected[link.first] = connected[link.second] = true;
            }
        }
    }
    check(std::all_of(connected.begin(), connected.end(), [](bool value) { return value; }),
          "reflected grid patches must remain connected through wall seams");
    const auto triple = viewer::make_wavefront(config, 50, 1, 1);
    const double contact = std::sqrt(50.0) / 1480.0;
    const auto exact = triple.sample_at_time(contact);
    near(exact.points[0].position_m.x, 10, "simultaneous corner contact x");
    near(exact.points[0].position_m.y, 10, "simultaneous corner contact y");
    const auto beyond = triple.sample_at_time(contact + 0.1 / 1480.0);
    check(beyond.points[0].position_m.x < 10 && beyond.points[0].position_m.y < 10,
          "both axes must reflect after a simultaneous corner contact");
}

void independent_geometry_and_pose() {
    sonar::Config config;
    auto reference = viewer::make_wavefront(config, 0, 3, 3);
    config.settings.max_outward_hits = 1;
    config.pool.wall_reflection = 0;
    config.pool.floor_reflection = 0;
    config.pool.surface_reflection = 0;
    config.settings.volume_scatterers = 4096;
    config.settings.surface_model = sonar::SurfaceModel::Roughness;
    auto independent = viewer::make_wavefront(config, 0, 3, 3);
    const auto a = reference.sample_at_time(18.0 / 1480.0);
    const auto b = independent.sample_at_time(18.0 / 1480.0);
    check(a.points.size() == b.points.size(), "materials and echo budget must not erase geometry");
    for (std::size_t i = 0; i < a.points.size(); ++i) {
        near(sonar::length(a.points[i].position_m - b.points[i].position_m), 0,
             "ideal-mirror front is independent of echo energy and particles");
    }
    config = sonar::Config{};
    config.pose = {{4, 2, 1}, 23, -17, 41};
    config.settings.horizontal_rays = 3;
    config.settings.vertical_rays = 3;
    auto posed = viewer::make_wavefront(config, 23, 3, 3);
    const auto ping = sonar::simulate_ping(config, 23);
    for (const auto &echo : ping.echoes) {
        if (echo.outward_hits == 1) {
            const auto delta = echo.path[1].position_m - config.pose.position_m;
            near(sonar::length(posed.rays[echo.ray_id].direction -
                               delta * (1.0 / sonar::length(delta))),
                 0, "geometric launch transform must agree with the numerical core");
        }
    }
    config = sonar::Config{};
    config.pool = {100, 100, 100, 0.8, 0.7, 0.95};
    config.pose.position_m = {50, 50, 50};
    const auto no_contact = viewer::make_wavefront(config, 0, 3, 3);
    check(sonar::simulate_ping(config, 0).echoes.empty(),
          "no-contact fixture should have no echoes");
    check(no_contact.sample_at_time(0).points.size() == 9 &&
              no_contact.sample_at_time(no_contact.duration_s()).points.size() == 9,
          "the complete front must exist even when no surface lies within range");
}

void dense_front_continuity() {
    sonar::Config config;
    config.pose = {{4, 2, 1}, 13, -9, 31};
    const auto dense = viewer::make_wavefront(config, 37);
    check(dense.rays.size() == 6144,
          "default grid must substantially increase vertical resolution");
    auto previous = dense.sample_at_time(0);
    const double step_m = dense.max_range_m / 100;
    for (unsigned step = 1; step <= 100; ++step) {
        const auto frame = dense.sample_at_time(dense.duration_s() * step / 100);
        check(frame.points.size() >= dense.rays.size(),
              "every ray must remain present throughout the complete playback window");
        inside(frame, config);
        for (std::size_t ray = 0; ray < dense.rays.size(); ++ray) {
            check(frame.points[ray].ray_id == ray,
                  "stable ray indices must precede any interpolated seam vertices");
            check(sonar::length(frame.points[ray].position_m - previous.points[ray].position_m) <=
                      step_m + 1e-9,
                  "ray trajectories cannot jump faster than sound speed at reflections");
        }
        previous = frame;
    }
    config.pool = {0.01, 0.01, 0.01, 0.8, 0.7, 0.95};
    config.pose = {{0.005, 0.005, 0.005}, 37, 22, 14};
    config.settings.max_range_m = 50;
    const auto many_bounces = viewer::make_wavefront(config, 0, 1, 2);
    const auto frame = many_bounces.sample_at_time(many_bounces.duration_s());
    check(frame.points.size() > 100, "small-pool fixture should exercise many folded chord seams");
    inside(frame, config);
}

} // namespace

int main() {
    try {
        normal_travel_and_clock();
        topology_and_wall_seams();
        independent_geometry_and_pose();
        dense_front_continuity();
        std::cout << "PASS: full-window geometric travel, continuous reflections, wall seams, "
                     "pose, energy independence, dense sampling and bounds\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
