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

void roughness_surface_model() {
    auto config = single_ray();
    config.settings.surface_model = sonar::SurfaceModel::Roughness;
    config.settings.surface_roughness_m = 0;
    const auto normal = sonar::surface_response(config.settings, 1);
    const auto oblique = sonar::surface_response(config.settings, std::sqrt(0.5));
    near(normal.specular_return, 1, "smooth normal surface returns coherently");
    check(oblique.specular_return < 1e-100 && oblique.diffuse_return == 0,
          "smooth oblique reflection misses the reverse incident direction");
    const auto ping = sonar::simulate_ping(config, 0);
    check(ping.echoes.size() == 1 && ping.echoes[0].relative_intensity > 0,
          "normal specular return is consumed instead of double-counted as continuation");
    double previous_diffuse = 0;
    for (double sigma : {0.0, 0.00005, 0.0001, 0.0005}) {
        config.settings.surface_roughness_m = sigma;
        const auto response = sonar::surface_response(config.settings, 0.7);
        check(response.diffuse_return >= previous_diffuse,
              "diffuse share increases as roughness grows at fixed frequency and angle");
        previous_diffuse = response.diffuse_return;
        for (double mu : {0.0, 0.01, 0.5, 0.99, 1.0}) {
            const auto shares = sonar::surface_response(config.settings, mu);
            check(shares.specular_return + shares.diffuse_return + shares.continuation <= 1 + 1e-12,
                  "received plus continuing energy must not exceed reflected budget");
        }
    }
    config.settings.surface_model = sonar::SurfaceModel::Legacy;
    const auto legacy = sonar::simulate_ping(config, 0);
    const auto baseline = sonar::simulate_ping(single_ray(), 0);
    near(legacy.echoes[0].relative_intensity, baseline.echoes[0].relative_intensity,
         "disabled roughness preserves the legacy baseline");
    std::istringstream valid("surface_model=roughness\nfrequency_hz=750000\nsurface_roughness_m=0."
                             "0002\nvolume_scatterers=4\nvolume_seed=7\nvolume_strength=0.01\n");
    const auto loaded = sonar::read_config(valid);
    check(loaded.settings.surface_model == sonar::SurfaceModel::Roughness &&
              loaded.settings.volume_seed == 7,
          "scattering configuration is parsed");
    for (const auto *invalid : {"surface_model=mirror", "surface_roughness_m=-1", "frequency_hz=0",
                                "volume_scatterers=4097", "volume_strength=nan"}) {
        rejects(
            [&] {
                std::istringstream input(invalid);
                sonar::read_config(input);
            },
            invalid);
    }
}

void volume_scattering() {
    auto config = single_ray();
    config.settings.volume_scatterers = 32;
    config.settings.volume_strength = 0.01;
    const auto cloud = sonar::volume_points(config);
    const auto repeated = sonar::volume_points(config);
    check(cloud.size() == 32, "point count matches configuration");
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        near(sonar::length(cloud[index] - repeated[index]), 0, "seed reproduces point positions");
        check(cloud[index].x > 0 && cloud[index].x < config.pool.length_m && cloud[index].y > 0 &&
                  cloud[index].y < config.pool.width_m && cloud[index].z > 0 &&
                  cloud[index].z < config.pool.depth_m,
              "particles remain strictly inside the pool");
    }
    auto changed = config;
    ++changed.settings.volume_seed;
    check(sonar::length(sonar::volume_points(changed)[0] - cloud[0]) > 0.01,
          "another seed changes the cloud");
    config.pose.position_m = cloud[0];
    const bool positive = cloud[0].x > 1.0;
    config.pose.position_m.x += positive ? -1.0 : 1.0;
    config.pose.yaw_deg = positive ? 0 : 180;
    const auto find_particle = [](const sonar::Ping &ping) -> const sonar::Echo & {
        for (const auto &echo : ping.echoes) {
            if (echo.kind == sonar::EchoKind::Volume && echo.scatterer_id == 0) {
                return echo;
            }
        }
        throw std::runtime_error("expected aligned particle was absent");
    };
    const auto ping = sonar::simulate_ping(config, 0);
    const auto &volume = find_particle(ping);
    check(!volume.is_multipath() && volume.path[1].volume_scatter,
          "volume has its own kind and vertex role");
    near(volume.apparent_range_m, 1, "single-scatter range");
    near(volume.total_path_m, 2, "single-scatter closed path");
    near(volume.travel_time_s, 2 / config.settings.sound_speed_m_s, "single-scatter timing");
    near(volume.relative_intensity, 0.01 * std::exp(-0.01 * 2),
         "isotropic relative particle energy");
    double received_volume = 0;
    for (const auto &bin : ping.profile) {
        received_volume += bin.volume_intensity;
    }
    check(received_volume > 0, "particle returns accumulate in their separate profile channel");
    config.settings.horizontal_rays = 9;
    config.settings.vertical_rays = 7;
    near(find_particle(sonar::simulate_ping(config, 0)).relative_intensity,
         volume.relative_intensity, "particle energy does not depend on quadrature ray count");
    const auto away = sonar::simulate_ping(config, 100);
    check(std::none_of(away.echoes.begin(), away.echoes.end(),
                       [](const auto &echo) {
                           return echo.kind == sonar::EchoKind::Volume && echo.scatterer_id == 0;
                       }),
          "point outside the continuous finite fan is rejected");
    config.settings.volume_strength = 0;
    const auto disabled = sonar::simulate_ping(config, 0);
    check(std::none_of(disabled.echoes.begin(), disabled.echoes.end(),
                       [](const auto &echo) { return echo.kind == sonar::EchoKind::Volume; }),
          "zero strength disables volume echoes");
    for (const auto &bin : disabled.profile) {
        near(bin.volume_intensity, 0, "zero volume channel");
    }
    config.settings.volume_strength = 0.01;
    config.pose.position_m = cloud[0];
    const bool upward = cloud[0].z > 1.0;
    config.pose.position_m.z += upward ? -1.0 : 1.0;
    config.pose.yaw_deg = 0;
    config.pose.pitch_deg = upward ? -90.0 : 90.0;
    const auto vertical = sonar::simulate_ping(config, 0);
    near(find_particle(vertical).apparent_range_m, 1,
         "inverse pose transform accepts a vertical beam");
    const auto vertical_energy = find_particle(vertical).relative_intensity;
    config.pool.wall_reflection = 0;
    config.pool.floor_reflection = 0;
    config.pool.surface_reflection = 0;
    const auto particles_only = sonar::simulate_ping(config, 0);
    near(find_particle(particles_only).relative_intensity, vertical_energy,
         "volume scattering is independent of wall reflection coefficients");
    near(profile_energy(particles_only, false) + profile_energy(particles_only, true), 0,
         "volume energy is not mislabeled as direct or multipath surface energy");
    config.settings.max_range_m = 0.75;
    const auto limited = sonar::simulate_ping(config, 0);
    check(std::none_of(limited.echoes.begin(), limited.echoes.end(),
                       [](const auto &echo) {
                           return echo.kind == sonar::EchoKind::Volume && echo.scatterer_id == 0;
                       }),
          "volume obeys maximum range");
    config.settings.max_range_m = 20;
    config.settings.volume_scatterers = 0;
    const auto no_particles = sonar::simulate_ping(config, 0);
    for (const auto &bin : no_particles.profile) {
        near(bin.volume_intensity, 0, "zero count disables volume channel");
    }
}

sonar::Config receiver_fixture() {
    auto config = single_ray();
    config.settings.return_model = sonar::ReturnModel::ReceiverPaths;
    config.settings.surface_model = sonar::SurfaceModel::Roughness;
    config.settings.surface_roughness_m = 0;
    config.settings.max_outward_hits = 1;
    return config;
}

void receiver_specular_geometry() {
    auto config = receiver_fixture();
    const auto ping = sonar::simulate_ping(config, 0);
    check(ping.echoes.size() == 1, "smooth normal wall produces one coherent receiver capture");
    const auto &echo = ping.echoes[0];
    check(echo.return_kind == sonar::ReturnKind::SpecularReceiver && echo.path.size() == 3 &&
              echo.interaction_count == 1 && !echo.is_multipath(),
          "capture stores its actual reflected route");
    near(echo.total_path_m, 10, "normal specular path has a center-to-center round trip");
    near(echo.outbound_path_m, 5, "outbound distance to reflecting wall");
    near(echo.return_path_m, 5, "return distance ends at the receiver center");
    near(sonar::length(echo.path.back().position_m - config.pose.position_m), 0,
         "direct capture terminates at the receiver center");
    near(echo.travel_time_s, 10 / config.settings.sound_speed_m_s, "captured route timing");
    near(echo.arrival_azimuth_deg, 0, "arrival bearing faces the emitting head");
    config.pose.yaw_deg = 30;
    check(sonar::simulate_ping(config, 0).echoes.empty(),
          "smooth oblique reflection that misses aperture produces no fake echo");
    config.pose.yaw_deg = 0;
    config.pool.wall_reflection = 0;
    check(sonar::simulate_ping(config, 0).echoes.empty(),
          "absorbing wall produces no received energy");
    config.pool.wall_reflection = 0.8;
    config.settings.max_range_m = 4.99;
    check(sonar::simulate_ping(config, 0).echoes.empty(),
          "total roundtrip beyond 2Rmax is rejected");
    config.settings.max_range_m = 5.01;
    const auto within = sonar::simulate_ping(config, 0);
    check(within.echoes.size() == 1 &&
              within.echoes[0].in_receive_window,
          "an arrival just inside the range window contributes to the profile");
}

void specular_sampling_and_timing() {
    auto config = receiver_fixture();
    config.settings.horizontal_rays = 3;
    config.settings.vertical_rays = 11;
    const auto odd = sonar::simulate_ping(config, 0);
    config.settings.vertical_rays = 12;
    const auto even = sonar::simulate_ping(config, 0);
    check(odd.echoes.size() == 1 && even.echoes.size() == 1,
          "direct specular capture must survive an even grid that misses the wall normal");
    check(odd.echoes[0].relative_intensity == even.echoes[0].relative_intensity,
          "direct specular energy is independent of the angular grid");

    config = receiver_fixture();
    config.settings.max_outward_hits = 3;
    for (const double radius : {0.025, 0.1}) {
        config.settings.receiver_radius_m = radius;
        const auto ping = sonar::simulate_ping(config, 0);
        const auto multi = std::find_if(ping.echoes.begin(), ping.echoes.end(),
                                        [](const auto &echo) { return echo.is_multipath(); });
        check(multi != ping.echoes.end() && multi->interaction_count == 3,
              "normal ray can return again after three wall contacts");
        near(multi->total_path_m, 30, "higher-order arrival uses closest approach, not sphere entry");
        near(multi->apparent_range_m, 15, "capture radius does not shorten the apparent range");
    }
}

void receiver_diffuse_routes() {
    auto config = receiver_fixture();
    config.pose.position_m.z = 0.4;
    config.settings.surface_roughness_m = 0.00015;
    config.settings.max_outward_hits = 2;
    const auto ping = sonar::simulate_ping(config, 0);
    const sonar::Echo *direct = nullptr;
    const sonar::Echo *via_floor = nullptr;
    for (const auto &echo : ping.echoes) {
        if (echo.return_kind == sonar::ReturnKind::DiffuseReceiver && echo.outward_hits == 1) {
            if (echo.path.size() == 3) {
                direct = &echo;
            }
            if (echo.path.size() == 4 && echo.path[2].surface_mask == (1U << 4)) {
                via_floor = &echo;
            }
        }
    }
    check(direct && via_floor,
          "one launched ping and scatter hit produce direct and reflected receiver connections");
    near(direct->total_path_m, 10, "direct diffuse receiver route");
    near(via_floor->total_path_m, 5 + std::sqrt(25 + 0.64),
         "image-source floor return has its own longer length");
    check(via_floor->return_start_vertex == 1 && via_floor->interaction_count == 2 &&
              via_floor->is_multipath(),
          "one outbound hit plus a return reflection is multipath");
    near(via_floor->path[2].position_m.x, 7.5, "floor return image-source contact x");
    near(via_floor->path[2].position_m.z, 0, "floor return contacts actual boundary");
    const auto in = via_floor->path[2].position_m - via_floor->path[1].position_m;
    const auto out = via_floor->path[3].position_m - via_floor->path[2].position_m;
    near(sonar::length(sonar::reflect(in * (1 / sonar::length(in)), {0, 0, 1}) -
                       out * (1 / sonar::length(out))),
         0, "image return obeys the specular reflection law");
    check(via_floor->outbound_path_m != via_floor->return_path_m,
          "return is not forced to retrace or have the outbound length");
    const double direct_energy = direct->relative_intensity;
    config.settings.surface_roughness_m = 0.0003;
    const auto rougher = sonar::simulate_ping(config, 0);
    check(std::any_of(rougher.echoes.begin(), rougher.echoes.end(),
                      [&](const auto &echo) {
                          return echo.return_kind == sonar::ReturnKind::DiffuseReceiver &&
                                 echo.path.size() == 3 && echo.relative_intensity > direct_energy;
                      }),
          "rougher terminal wall increases direct diffuse energy in the fixture");
    config.settings.max_outward_hits = 1;
    const auto limited = sonar::simulate_ping(config, 0);
    check(std::none_of(limited.echoes.begin(), limited.echoes.end(),
                       [](const auto &echo) { return echo.interaction_count > 1; }),
          "wall budget covers both outbound and return interactions");
    config.settings.max_outward_hits = 2;
    config.pose.position_m.z = 1.5;
    const auto narrow_rx = sonar::simulate_ping(config, 0);
    check(std::none_of(narrow_rx.echoes.begin(), narrow_rx.echoes.end(),
                       [](const auto &echo) {
                           return echo.return_kind == sonar::ReturnKind::DiffuseReceiver &&
                                  echo.outward_hits == 1 && echo.path.size() == 4 &&
                                  echo.path[2].surface_mask == (1U << 4);
                       }),
          "geometric floor connection outside the 25 degree receive fan is rejected");
    config.settings.return_reflections = 0;
    config.pose.position_m.z = 0.4;
    const auto no_return_wall = sonar::simulate_ping(config, 0);
    check(std::none_of(no_return_wall.echoes.begin(), no_return_wall.echoes.end(),
                       [](const auto &echo) {
                           return echo.return_kind == sonar::ReturnKind::DiffuseReceiver &&
                                  echo.path.size() > 3;
                       }),
          "return_reflections=0 disables image-source branches");
}

void receiver_path_invariants() {
    auto config = receiver_fixture();
    config.settings.horizontal_rays = 3;
    config.settings.vertical_rays = 11;
    config.settings.surface_roughness_m = 0.00015;
    config.settings.max_outward_hits = 4;
    for (unsigned angle = 0; angle < 400; ++angle) {
        const auto ping = sonar::simulate_ping(config, angle);
        for (const auto &echo : ping.echoes) {
            double measured = 0;
            for (std::size_t vertex = 1; vertex < echo.path.size(); ++vertex) {
                measured +=
                    sonar::length(echo.path[vertex].position_m - echo.path[vertex - 1].position_m);
                const auto point = echo.path[vertex].position_m;
                check(point.x >= -1e-9 && point.x <= config.pool.length_m + 1e-9 &&
                          point.y >= -1e-9 && point.y <= config.pool.width_m + 1e-9 &&
                          point.z >= -1e-9 && point.z <= config.pool.depth_m + 1e-9,
                      "received route vertices remain inside the pool");
            }
            near(measured, echo.total_path_m, "stored path length is the propagated route");
            near(echo.outbound_path_m + echo.return_path_m, echo.total_path_m,
                 "outbound and return lengths sum correctly");
            near(echo.travel_time_s * config.settings.sound_speed_m_s, echo.total_path_m,
                 "all received route times use actual length");
            check(echo.relative_intensity > 0 && std::isfinite(echo.relative_intensity) &&
                      echo.total_path_m <= 2 * config.settings.max_range_m &&
                      echo.interaction_count <= config.settings.max_outward_hits,
                  "new receiver mode stores only positive finite, bounded received routes");
            const double arrival_delta =
                std::remainder(echo.arrival_azimuth_deg - angle * 0.9, 360.0);
            check(std::abs(arrival_delta) <= 1 + 1e-9 &&
                      std::abs(echo.arrival_elevation_deg) <= 12.5 + 1e-9,
                  "received surface routes pass both receiver angular gates");
        }
    }
    std::istringstream input(
        "return_model=receiver_paths\nreceiver_radius_m=0.02\nreturn_reflections=1\n");
    const auto loaded = sonar::read_config(input);
    check(loaded.settings.return_model == sonar::ReturnModel::ReceiverPaths,
          "receiver model config is parsed");
    for (const char *text : {"return_model=unknown", "return_reflections=2", "receiver_radius_m=0",
                             "return_model=receiver_paths\nsonar_z_m=0.04"}) {
        rejects(
            [&] {
                std::istringstream invalid(text);
                sonar::read_config(invalid);
            },
            text);
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
        {"roughness surface model", roughness_surface_model},
        {"volume scattering", volume_scattering},
        {"receiver specular geometry", receiver_specular_geometry},
        {"specular sampling and timing", specular_sampling_and_timing},
        {"receiver diffuse routes", receiver_diffuse_routes},
        {"receiver path invariants", receiver_path_invariants},
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
