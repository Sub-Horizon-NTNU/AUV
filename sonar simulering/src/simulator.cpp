#include "sonar/simulator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

namespace sonar {
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double degrees_to_radians = pi / 180.0;
constexpr double reference_range_m = 1.0;

void require_range(double value, double lower, double upper, const char *name) {
    if (!std::isfinite(value) || value < lower || value > upper) {
        throw std::invalid_argument(std::string(name) + " must be finite and in [" +
                                    std::to_string(lower) + ", " + std::to_string(upper) + "]");
    }
}

void require_count(unsigned value, unsigned lower, unsigned upper, const char *name) {
    if (value < lower || value > upper) {
        throw std::invalid_argument(std::string(name) + " must be in [" + std::to_string(lower) +
                                    ", " + std::to_string(upper) + "]");
    }
}

double coordinate(Vec3 value, unsigned axis) {
    switch (axis) {
    case 0:
        return value.x;
    case 1:
        return value.y;
    default:
        return value.z;
    }
}

Vec3 rotate(Vec3 direction, const Pose &pose) {
    const double roll = pose.roll_deg * degrees_to_radians;
    const double pitch = pose.pitch_deg * degrees_to_radians;
    const double yaw = pose.yaw_deg * degrees_to_radians;
    const Vec3 rolled{direction.x, std::cos(roll) * direction.y - std::sin(roll) * direction.z,
                      std::sin(roll) * direction.y + std::cos(roll) * direction.z};
    const Vec3 pitched{std::cos(pitch) * rolled.x + std::sin(pitch) * rolled.z, rolled.y,
                       -std::sin(pitch) * rolled.x + std::cos(pitch) * rolled.z};
    return {std::cos(yaw) * pitched.x - std::sin(yaw) * pitched.y,
            std::sin(yaw) * pitched.x + std::cos(yaw) * pitched.y, pitched.z};
}

struct Hit {
    Vec3 position_m;
    double distance_m{};
    std::uint8_t mask{};
    Surface primary{};
};

Hit next_hit(Vec3 origin, Vec3 direction, const Pool &pool) {
    const std::array<double, 3> dimensions{pool.length_m, pool.width_m, pool.depth_m};
    std::array<double, 3> distances{};
    std::array<unsigned, 3> faces{};
    distances.fill(std::numeric_limits<double>::infinity());
    for (unsigned axis = 0; axis < 3; ++axis) {
        const double component = coordinate(direction, axis);
        if (std::abs(component) < 1e-14) {
            continue;
        }
        const double boundary = component > 0.0 ? dimensions[axis] : 0.0;
        distances[axis] = (boundary - coordinate(origin, axis)) / component;
        faces[axis] = 2 * axis + (component > 0.0 ? 1U : 0U);
    }

    const double nearest = *std::min_element(distances.begin(), distances.end());
    const double tie_tolerance = 1e-10 * std::max(1.0, nearest);
    Hit result{origin + direction * nearest, nearest, 0, Surface::WaterSurface};
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (std::abs(distances[axis] - nearest) <= tie_tolerance) {
            result.mask = static_cast<std::uint8_t>(result.mask | (1U << faces[axis]));
            if (faces[axis] < static_cast<unsigned>(result.primary)) {
                result.primary = static_cast<Surface>(faces[axis]);
            }
            // Snap simultaneous contacts to exact planes, avoiding tiny negative next steps.
            if (axis == 0) {
                result.position_m.x = direction.x > 0.0 ? pool.length_m : 0.0;
            } else if (axis == 1) {
                result.position_m.y = direction.y > 0.0 ? pool.width_m : 0.0;
            } else {
                result.position_m.z = direction.z > 0.0 ? pool.depth_m : 0.0;
            }
        }
    }
    return result;
}

double reflection_coefficient(Surface surface, const Pool &pool) {
    if (surface == Surface::Floor) {
        return pool.floor_reflection;
    }
    if (surface == Surface::WaterSurface) {
        return pool.surface_reflection;
    }
    return pool.wall_reflection;
}

double fan_offset(unsigned index, unsigned count, double width_deg) {
    return ((static_cast<double>(index) + 0.5) / static_cast<double>(count) - 0.5) * width_deg;
}

void accumulate_profile(const Config &config, const Echo &echo, std::vector<RangeBin> &profile) {
    if (!echo.in_receive_window) {
        return;
    }
    const double bin_width = config.settings.max_range_m / static_cast<double>(profile.size());
    // A causal rectangular transmitted pulse spreads the return after its geometric arrival.
    const double pulse_width_m =
        config.settings.sound_speed_m_s * config.settings.transmit_duration_us * 1e-6 / 2.0;
    const double start = echo.apparent_range_m;
    const double end = start + pulse_width_m;
    const auto first_bin = static_cast<std::size_t>(start / bin_width);
    for (std::size_t index = first_bin; index < profile.size(); ++index) {
        const double lower = static_cast<double>(index) * bin_width;
        if (lower >= end) {
            break;
        }
        const double upper = lower + bin_width;
        const double overlap =
            std::max(0.0, std::min(end, upper) - std::max({start, lower, minimum_range_m}));
        const double contribution = echo.relative_intensity * overlap / pulse_width_m;
        if (echo.kind == EchoKind::Volume) {
            profile[index].volume_intensity += contribution;
        } else if (echo.is_multipath()) {
            profile[index].multipath_intensity += contribution;
        } else {
            profile[index].direct_intensity += contribution;
        }
    }
}

void trace_ray(const Config &config, Vec3 direction, unsigned ray_id, double azimuth_deg,
               double elevation_deg, double ray_weight, Ping &ping) {
    std::vector<PathVertex> outward_path{{config.pose.position_m, 0}};
    Vec3 current = config.pose.position_m;
    double outward_distance = 0.0;
    double specular_energy = 1.0;

    for (unsigned hit_index = 1; hit_index <= config.settings.max_outward_hits; ++hit_index) {
        const Hit hit = next_hit(current, direction, config.pool);
        outward_distance += hit.distance_m;
        if (outward_distance > config.settings.max_range_m) {
            break;
        }
        outward_path.push_back({hit.position_m, hit.mask});
        const unsigned primary_axis = static_cast<unsigned>(hit.primary) / 2;
        const double incidence_cosine = std::abs(coordinate(direction, primary_axis));
        const auto response = surface_response(config.settings, incidence_cosine);
        const double terminal_energy = reflection_coefficient(hit.primary, config.pool) *
                                       (response.specular_return + response.diffuse_return);
        const double spreading_range = std::max(reference_range_m, outward_distance);
        // Two-way energy spreading (r_ref/r)^4, clipped inside the arbitrary 1 m reference.
        const double relative_intensity =
            ray_weight * specular_energy * specular_energy * terminal_energy *
            std::pow(reference_range_m / spreading_range, 4.0) *
            std::exp(-config.settings.absorption_per_m * 2.0 * outward_distance);
        // Squared continuation is retained from the restricted reciprocal model. With
        // receiver-aligned coherent energy consumed at earlier vertices, this is an
        // effective path-weight heuristic, not an exact transport integral.

        Echo echo{ray_id,
                  hit_index,
                  2 * hit_index - 1,
                  azimuth_deg,
                  elevation_deg,
                  2.0 * outward_distance,
                  2.0 * outward_distance / config.settings.sound_speed_m_s,
                  outward_distance,
                  relative_intensity,
                  outward_distance >= minimum_range_m &&
                      outward_distance < config.settings.max_range_m,
                  outward_path};
        for (std::size_t index = outward_path.size() - 1; index > 0; --index) {
            echo.path.push_back(outward_path[index - 1]);
        }
        accumulate_profile(config, echo, ping.profile);
        ping.echoes.push_back(std::move(echo));

        // Edge/corner ties use the lowest-numbered face for backscatter. Continuation
        // reflects across every tied plane; no edge diffraction is represented.
        for (unsigned face = 0; face < 6; ++face) {
            if ((hit.mask & (1U << face)) == 0) {
                continue;
            }
            const auto continuing =
                surface_response(config.settings, std::abs(coordinate(direction, face / 2)));
            specular_energy *= reflection_coefficient(static_cast<Surface>(face), config.pool) *
                               continuing.continuation;
            Vec3 normal{};
            if (face / 2 == 0) {
                normal.x = 1.0;
            } else if (face / 2 == 1) {
                normal.y = 1.0;
            } else {
                normal.z = 1.0;
            }
            direction = reflect(direction, normal);
        }
        current = hit.position_m;
        if (specular_energy == 0.0) {
            break;
        }
    }
}

void scatter_volume(const Config &config, unsigned angle_grad, Ping &ping) {
    if (config.settings.volume_scatterers == 0 || config.settings.volume_strength == 0.0) {
        return;
    }
    const auto points = volume_points(config);
    // Dot with local basis vectors is the inverse of the core's orthogonal pose rotation.
    const Vec3 basis_x = rotate({1, 0, 0}, config.pose);
    const Vec3 basis_y = rotate({0, 1, 0}, config.pose);
    const Vec3 basis_z = rotate({0, 0, 1}, config.pose);
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto delta = points[index] - config.pose.position_m;
        const double range = length(delta);
        if (range < minimum_range_m || range >= config.settings.max_range_m) {
            continue;
        }
        const Vec3 local{dot(delta, basis_x), dot(delta, basis_y), dot(delta, basis_z)};
        const double azimuth = std::atan2(local.y, local.x) / degrees_to_radians;
        const double elevation =
            std::atan2(local.z, std::hypot(local.x, local.y)) / degrees_to_radians;
        const double difference = std::remainder(azimuth - angle_grad * 0.9, 360.0);
        if (std::abs(difference) > horizontal_beam_width_deg / 2 ||
            std::abs(elevation) > vertical_beam_width_deg / 2) {
            continue;
        }
        // Unit top-hat TX/RX gain inside the finite beam, independent of ray quadrature.
        // Strength is an uncalibrated per-particle coefficient, not a bulk water property.
        const double intensity =
            config.settings.volume_strength *
            std::pow(reference_range_m / std::max(reference_range_m, range), 4.0) *
            std::exp(-config.settings.absorption_per_m * 2 * range);
        Echo echo{std::numeric_limits<unsigned>::max(),
                  1,
                  1,
                  azimuth,
                  elevation,
                  2 * range,
                  2 * range / config.settings.sound_speed_m_s,
                  range,
                  intensity,
                  true,
                  {{config.pose.position_m, 0, false},
                   {points[index], 0, true},
                   {config.pose.position_m, 0, false}},
                  EchoKind::Volume,
                  static_cast<unsigned>(index)};
        accumulate_profile(config, echo, ping.profile);
        ping.echoes.push_back(std::move(echo));
    }
}

} // namespace

SurfaceResponse surface_response(const Settings &settings, double incidence_cosine) {
    const double mu = std::clamp(incidence_cosine, 0.0, 1.0);
    if (settings.surface_model == SurfaceModel::Legacy) {
        return {0.0, 0.0, settings.backscatter_fraction * mu * mu,
                1.0 - settings.backscatter_fraction};
    }
    const double wavelength = settings.sound_speed_m_s / settings.frequency_hz;
    const double rayleigh = 4 * pi * settings.surface_roughness_m * mu / wavelength;
    const double coherent = std::exp(-rayleigh * rayleigh);
    // Angle between specularly reflected direction and reverse incident direction.
    const double mismatch = std::acos(std::clamp(2 * mu * mu - 1.0, -1.0, 1.0));
    constexpr double receiver_lobe_width_rad = degrees_to_radians; // Illustrative 1 degree sigma.
    const double lobe = std::exp(-0.5 * std::pow(mismatch / receiver_lobe_width_rad, 2.0));
    return {coherent, coherent * lobe, (1.0 - coherent) * settings.backscatter_fraction * mu * mu,
            coherent * (1.0 - lobe)};
}

std::vector<Vec3> volume_points(const Config &config) {
    validate(config);
    std::mt19937 generator(config.settings.volume_seed);
    const auto unit = [&generator] {
        return (static_cast<double>(generator()) + 0.5) / 4294967296.0;
    };
    std::vector<Vec3> points;
    points.reserve(config.settings.volume_scatterers);
    for (unsigned index = 0; index < config.settings.volume_scatterers; ++index) {
        points.push_back({unit() * config.pool.length_m, unit() * config.pool.width_m,
                          unit() * config.pool.depth_m});
    }
    return points;
}

Vec3 operator+(Vec3 a, Vec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(Vec3 a, Vec3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(Vec3 a, double scale) {
    return {a.x * scale, a.y * scale, a.z * scale};
}

double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double length(Vec3 a) {
    return std::sqrt(dot(a, a));
}

Vec3 reflect(Vec3 direction, Vec3 unit_normal) {
    return direction - unit_normal * (2.0 * dot(direction, unit_normal));
}

const char *surface_name(Surface surface) {
    switch (surface) {
    case Surface::XMin:
        return "x_min";
    case Surface::XMax:
        return "x_max";
    case Surface::YMin:
        return "y_min";
    case Surface::YMax:
        return "y_max";
    case Surface::Floor:
        return "floor";
    case Surface::WaterSurface:
        return "water_surface";
    }
    return "unknown";
}

void validate(const Config &config) {
    require_range(config.pool.length_m, 0.01, 10000.0, "pool_length_m");
    require_range(config.pool.width_m, 0.01, 10000.0, "pool_width_m");
    require_range(config.pool.depth_m, 0.01, 10000.0, "pool_depth_m");
    const std::array<double, 3> dimensions{config.pool.length_m, config.pool.width_m,
                                           config.pool.depth_m};
    const std::array<const char *, 3> position_names{"sonar_x_m", "sonar_y_m", "sonar_z_m"};
    for (unsigned axis = 0; axis < 3; ++axis) {
        const double position = coordinate(config.pose.position_m, axis);
        if (!std::isfinite(position) || position <= 0.0 || position >= dimensions[axis]) {
            throw std::invalid_argument(std::string(position_names[axis]) +
                                        " must be strictly inside the pool");
        }
    }
    require_range(config.pose.roll_deg, -360.0, 360.0, "sonar_roll_deg");
    require_range(config.pose.pitch_deg, -360.0, 360.0, "sonar_pitch_deg");
    require_range(config.pose.yaw_deg, -360.0, 360.0, "sonar_yaw_deg");
    require_range(config.pool.wall_reflection, 0.0, 1.0, "wall_reflection");
    require_range(config.pool.floor_reflection, 0.0, 1.0, "floor_reflection");
    require_range(config.pool.surface_reflection, 0.0, 1.0, "surface_reflection");
    require_range(config.settings.sound_speed_m_s, 100.0, 10000.0, "sound_speed_m_s");
    require_range(config.settings.max_range_m, minimum_range_m, 50.0, "max_range_m");
    require_range(config.settings.transmit_duration_us, 1.0, 1000.0, "transmit_duration_us");
    require_range(config.settings.absorption_per_m, 0.0, 100.0, "absorption_per_m");
    require_range(config.settings.backscatter_fraction, 0.0, 1.0, "backscatter_fraction");
    require_count(config.settings.horizontal_rays, 1, 101, "horizontal_rays");
    require_count(config.settings.vertical_rays, 1, 101, "vertical_rays");
    if (config.settings.horizontal_rays * config.settings.vertical_rays > 1024) {
        throw std::invalid_argument("horizontal_rays * vertical_rays must not exceed 1024");
    }
    require_count(config.settings.max_outward_hits, 1, 16, "max_outward_hits");
    require_count(config.settings.range_samples, 200, 1200, "range_samples");
    if (config.settings.surface_model != SurfaceModel::Legacy &&
        config.settings.surface_model != SurfaceModel::Roughness) {
        throw std::invalid_argument("unsupported surface_model");
    }
    require_range(config.settings.frequency_hz, 500000, 1000000, "frequency_hz");
    require_range(config.settings.surface_roughness_m, 0, 0.01, "surface_roughness_m");
    require_count(config.settings.volume_scatterers, 0, 4096, "volume_scatterers");
    require_range(config.settings.volume_strength, 0, 1, "volume_strength");
}

bool Echo::is_multipath() const {
    return kind == EchoKind::Surface && outward_hits > 1;
}

Ping simulate_ping(const Config &config, unsigned angle_grad) {
    validate(config);
    require_count(angle_grad, 0, 399, "angle_grad");
    Ping ping;
    ping.angle_grad = angle_grad;
    ping.profile.resize(config.settings.range_samples);
    const double bin_width = config.settings.max_range_m / config.settings.range_samples;
    for (std::size_t index = 0; index < ping.profile.size(); ++index) {
        ping.profile[index].center_range_m = (static_cast<double>(index) + 0.5) * bin_width;
    }

    double total_weight = 0.0;
    for (unsigned vertical = 0; vertical < config.settings.vertical_rays; ++vertical) {
        const double elevation =
            fan_offset(vertical, config.settings.vertical_rays, vertical_beam_width_deg);
        total_weight += std::cos(elevation * degrees_to_radians) * config.settings.horizontal_rays;
    }
    for (unsigned horizontal = 0; horizontal < config.settings.horizontal_rays; ++horizontal) {
        const double azimuth_deg =
            static_cast<double>(angle_grad) * 0.9 +
            fan_offset(horizontal, config.settings.horizontal_rays, horizontal_beam_width_deg);
        const double azimuth = azimuth_deg * degrees_to_radians;
        for (unsigned vertical = 0; vertical < config.settings.vertical_rays; ++vertical) {
            const double elevation_deg =
                fan_offset(vertical, config.settings.vertical_rays, vertical_beam_width_deg);
            const double elevation = elevation_deg * degrees_to_radians;
            const Vec3 local{std::cos(elevation) * std::cos(azimuth),
                             std::cos(elevation) * std::sin(azimuth), std::sin(elevation)};
            const Vec3 direction = rotate(local, config.pose);
            const unsigned ray_id = horizontal * config.settings.vertical_rays + vertical;
            trace_ray(config, direction, ray_id, azimuth_deg, elevation_deg,
                      std::cos(elevation) / total_weight, ping);
        }
    }
    scatter_volume(config, angle_grad, ping);
    return ping;
}

} // namespace sonar
