#include "wavefront.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace viewer {
namespace {

constexpr double radians = 3.14159265358979323846 / 180.0;
constexpr unsigned maximum_rays = 262144;
// With the explicit 100 ms horizon, c<=10000 m/s and sides>=0.01 m, an adjacent
// midpoint fan pair crosses fewer than 38000 mirror planes even at the coarsest
// vertical grid. Keep a fixed bound above that validated worst case.
constexpr std::size_t maximum_chord_splits = 65536;

double coordinate(sonar::Vec3 value, unsigned axis) {
    return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

sonar::Vec3 rotate(sonar::Vec3 value, const sonar::Pose &pose) {
    // Same local-to-pool Rz(yaw) Ry(pitch) Rx(roll) convention as the numerical core.
    const double roll = pose.roll_deg * radians;
    const double pitch = pose.pitch_deg * radians;
    const double yaw = pose.yaw_deg * radians;
    const sonar::Vec3 rolled{value.x, std::cos(roll) * value.y - std::sin(roll) * value.z,
                             std::sin(roll) * value.y + std::cos(roll) * value.z};
    const sonar::Vec3 pitched{std::cos(pitch) * rolled.x + std::sin(pitch) * rolled.z, rolled.y,
                              -std::sin(pitch) * rolled.x + std::cos(pitch) * rolled.z};
    return {std::cos(yaw) * pitched.x - std::sin(yaw) * pitched.y,
            std::sin(yaw) * pitched.x + std::cos(yaw) * pitched.y, pitched.z};
}

double fold(double unfolded, double dimension) {
    double phase = std::fmod(unfolded, 2 * dimension);
    if (phase < 0) {
        phase += 2 * dimension;
    }
    return phase <= dimension ? phase : 2 * dimension - phase;
}

sonar::Vec3 fold(sonar::Vec3 unfolded, sonar::Vec3 dimensions) {
    return {fold(unfolded.x, dimensions.x), fold(unfolded.y, dimensions.y),
            fold(unfolded.z, dimensions.z)};
}

unsigned crossings(sonar::Vec3 unfolded, sonar::Vec3 dimensions) {
    unsigned result = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const double cell = coordinate(unfolded, axis) / coordinate(dimensions, axis);
        // Count exact contacts consistently for either launch direction.
        result += static_cast<unsigned>(cell <= 0 ? 1 + std::floor(-cell) : std::floor(cell));
    }
    return result;
}

double reflected_component(double unfolded, double dimension, double incoming) {
    double cell = std::floor(unfolded / dimension);
    // At a wall use the outgoing side, including negative-direction exact contacts.
    if (incoming < 0 && unfolded / dimension == cell) {
        cell -= 1;
    }
    return std::fmod(std::abs(cell), 2.0) == 0 ? incoming : -incoming;
}

double transport_energy(const WavefrontRay &ray, sonar::Vec3 unfolded, sonar::Vec3 dimensions,
                        double traveled_m, double absorption_per_m, std::array<int, 3> &cells) {
    double log_energy = -absorption_per_m * traveled_m;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const double coordinate_m = coordinate(unfolded, axis);
        const double direction = coordinate(ray.direction, axis);
        const double dimension = coordinate(dimensions, axis);
        const double scaled = coordinate_m / dimension;
        const int cell = static_cast<int>(std::floor(scaled));
        cells[axis] = cell;
        unsigned contacts = 0;
        if (direction > 0 && cell > 0) {
            contacts = static_cast<unsigned>(cell);
        } else if (direction < 0 && coordinate_m <= 0) {
            contacts = static_cast<unsigned>(-cell) + (scaled == cell ? 1U : 0U);
        }
        // The first contacted face is selected by launch direction; low and high
        // walls alternate. Exact contacts include the outgoing reflection loss.
        const unsigned first = (contacts + 1) / 2;
        const unsigned second = contacts / 2;
        const unsigned low = direction < 0 ? first : second;
        const unsigned high = direction > 0 ? first : second;
        if (low) {
            log_energy += low * ray.log_face_survival[2 * axis];
        }
        if (high) {
            log_energy += high * ray.log_face_survival[2 * axis + 1];
        }
    }
    const double range = std::max(1.0, traveled_m);
    return std::exp(log_energy) / (range * range);
}

void append_folded_chord(WavefrontFrame &frame, std::size_t first, std::size_t second,
                         sonar::Vec3 start, sonar::Vec3 end, sonar::Vec3 dimensions,
                         std::vector<double> &splits) {
    const auto delta = end - start;
    splits.clear();
    // A chord in unfolded space is piecewise linear after folding. Insert each
    // mirror-plane crossing instead of joining two reflected branches through the pool.
    for (unsigned axis = 0; axis < 3; ++axis) {
        const double a = coordinate(start, axis);
        const double b = coordinate(end, axis);
        if (a == b) {
            continue;
        }
        const double dimension = coordinate(dimensions, axis);
        const auto lower = static_cast<int>(std::floor(std::min(a, b) / dimension)) + 1;
        const auto upper = static_cast<int>(std::ceil(std::max(a, b) / dimension)) - 1;
        for (int plane = lower; plane <= upper; ++plane) {
            splits.push_back((static_cast<double>(plane) * dimension - a) / (b - a));
            if (splits.size() > maximum_chord_splits) {
                throw std::runtime_error("geometric wavefront chord exceeds its seam budget");
            }
        }
    }
    std::sort(splits.begin(), splits.end());
    double previous = -1;
    std::size_t last = first;
    for (double fraction : splits) {
        if (fraction <= 0 || fraction >= 1 || std::abs(fraction - previous) < 1e-12) {
            continue;
        }
        const auto unfolded = start + delta * fraction;
        const auto index = frame.points.size();
        constexpr unsigned seam = std::numeric_limits<unsigned>::max();
        const double energy = frame.points[first].relative_energy * (1 - fraction) +
                              frame.points[second].relative_energy * fraction;
        frame.points.push_back({seam,
                                seam,
                                seam,
                                crossings(unfolded, dimensions),
                                fold(unfolded, dimensions),
                                {},
                                energy});
        frame.links.emplace_back(last, index);
        last = index;
        previous = fraction;
    }
    frame.links.emplace_back(last, second);
}

} // namespace

GeometricWavefront make_wavefront(const sonar::Config &config, unsigned angle_grad,
                                  unsigned horizontal_rays, unsigned vertical_rays) {
    sonar::validate(config);
    if (angle_grad > 399 || horizontal_rays == 0 || vertical_rays == 0 ||
        horizontal_rays > maximum_rays || vertical_rays > maximum_rays ||
        static_cast<std::size_t>(horizontal_rays) * vertical_rays > maximum_rays) {
        throw std::invalid_argument("wavefront requires angle 0..399 and 1..262144 total rays");
    }
    GeometricWavefront result{horizontal_rays,
                              vertical_rays,
                              config.settings.sound_speed_m_s,
                              config.settings.max_range_m,
                              config.pose.position_m,
                              {config.pool.length_m, config.pool.width_m, config.pool.depth_m},
                              {}};
    result.rays.reserve(static_cast<std::size_t>(horizontal_rays) * vertical_rays);
    result.absorption_per_m = config.settings.absorption_per_m;
    for (unsigned horizontal = 0; horizontal < horizontal_rays; ++horizontal) {
        const double azimuth = (angle_grad * 0.9 + ((horizontal + 0.5) / horizontal_rays - 0.5) *
                                                       sonar::horizontal_beam_width_deg) *
                               radians;
        for (unsigned vertical = 0; vertical < vertical_rays; ++vertical) {
            const double elevation =
                ((vertical + 0.5) / vertical_rays - 0.5) * sonar::vertical_beam_width_deg * radians;
            const sonar::Vec3 local{std::cos(elevation) * std::cos(azimuth),
                                    std::cos(elevation) * std::sin(azimuth), std::sin(elevation)};
            WavefrontRay ray{horizontal * vertical_rays + vertical, rotate(local, config.pose)};
            for (unsigned face = 0; face < 6; ++face) {
                const double mu = std::abs(coordinate(ray.direction, face / 2));
                const double material = face == 4   ? config.pool.floor_reflection
                                        : face == 5 ? config.pool.surface_reflection
                                                    : config.pool.wall_reflection;
                const double coherent =
                    config.settings.surface_model == sonar::SurfaceModel::Legacy
                        ? 1 - config.settings.backscatter_fraction
                        : sonar::surface_response(config.settings, mu).coherent_share;
                const double survival = material * coherent;
                ray.log_face_survival[face] =
                    survival > 0 ? std::log(survival) : -std::numeric_limits<double>::infinity();
            }
            result.rays.push_back(ray);
        }
    }
    return result;
}

double GeometricWavefront::duration_s(double horizon_s) const {
    if (!std::isfinite(horizon_s) || (horizon_s != 0 && (horizon_s < 0.001 || horizon_s > 0.1))) {
        throw std::invalid_argument(
            "explicit geometric wavefront horizon must be 0.001..0.1 seconds");
    }
    return horizon_s == 0 ? max_range_m / sound_speed_m_s : horizon_s;
}

WavefrontFrame GeometricWavefront::sample_at_time(double time_s, double horizon_s) const {
    WavefrontFrame frame;
    sample_into(time_s, frame, horizon_s);
    return frame;
}

void GeometricWavefront::sample_into(double time_s, WavefrontFrame &frame, double horizon_s) const {
    if (!std::isfinite(time_s) || time_s < 0) {
        throw std::invalid_argument("geometric wavefront time must be finite and nonnegative");
    }
    const double duration = duration_s(horizon_s);
    frame.points.clear();
    frame.links.clear();
    if (time_s > duration) {
        return;
    }
    const double traveled_m = sound_speed_m_s * time_s;
    std::vector<std::array<int, 3>> cells(rays.size());
    frame.points.reserve(rays.size());
    frame.links.reserve(2 * rays.size());
    for (const auto &ray : rays) {
        const auto position = origin_m + ray.direction * traveled_m;
        frame.points.push_back({ray.ray_id,
                                ray.ray_id / vertical_rays,
                                ray.ray_id % vertical_rays,
                                crossings(position, dimensions_m),
                                fold(position, dimensions_m),
                                {reflected_component(position.x, dimensions_m.x, ray.direction.x),
                                 reflected_component(position.y, dimensions_m.y, ray.direction.y),
                                 reflected_component(position.z, dimensions_m.z, ray.direction.z)},
                                transport_energy(ray, position, dimensions_m, traveled_m,
                                                 absorption_per_m, cells[ray.ray_id])});
    }
    // Reuse one seam scratch buffer for every neighbor edge in this frame.
    std::vector<double> splits;
    const auto connect = [&](std::size_t first, std::size_t second) {
        // A chord inside one unfolded box cell folds affinely: no seam work is
        // necessary. Most dense-grid neighbor pairs satisfy this condition.
        if (cells[first] == cells[second]) {
            frame.links.emplace_back(first, second);
        } else {
            append_folded_chord(frame, first, second, origin_m + rays[first].direction * traveled_m,
                                origin_m + rays[second].direction * traveled_m, dimensions_m,
                                splits);
        }
    };
    for (unsigned horizontal = 0; horizontal < horizontal_rays; ++horizontal) {
        for (unsigned vertical = 0; vertical < vertical_rays; ++vertical) {
            const std::size_t ray = static_cast<std::size_t>(horizontal) * vertical_rays + vertical;
            if (horizontal + 1 < horizontal_rays) {
                connect(ray, ray + vertical_rays);
            }
            if (vertical + 1 < vertical_rays) {
                connect(ray, ray + 1);
            }
        }
    }
}

} // namespace viewer
