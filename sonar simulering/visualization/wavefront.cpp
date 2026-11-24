#include "wavefront.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace viewer {
namespace {

constexpr double radians = 3.14159265358979323846 / 180.0;
constexpr unsigned maximum_rays = 262144;
// For validated range <= 50 m, box sides >= 0.01 m and the fixed fan, even
// the coarsest adjacent vertical pair crosses fewer than 2000 mirror planes.
constexpr std::size_t maximum_chord_splits = 4096;

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

void append_folded_chord(WavefrontFrame &frame, std::size_t first, std::size_t second,
                         sonar::Vec3 start, sonar::Vec3 end, sonar::Vec3 dimensions) {
    const auto delta = end - start;
    std::vector<double> splits;
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
        frame.points.push_back(
            {seam, seam, seam, crossings(unfolded, dimensions), fold(unfolded, dimensions)});
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
    for (unsigned horizontal = 0; horizontal < horizontal_rays; ++horizontal) {
        const double azimuth = (angle_grad * 0.9 + ((horizontal + 0.5) / horizontal_rays - 0.5) *
                                                       sonar::horizontal_beam_width_deg) *
                               radians;
        for (unsigned vertical = 0; vertical < vertical_rays; ++vertical) {
            const double elevation =
                ((vertical + 0.5) / vertical_rays - 0.5) * sonar::vertical_beam_width_deg * radians;
            const sonar::Vec3 local{std::cos(elevation) * std::cos(azimuth),
                                    std::cos(elevation) * std::sin(azimuth), std::sin(elevation)};
            result.rays.push_back(
                {horizontal * vertical_rays + vertical, rotate(local, config.pose)});
        }
    }
    return result;
}

double GeometricWavefront::duration_s() const {
    return max_range_m / sound_speed_m_s;
}

WavefrontFrame GeometricWavefront::sample_at_time(double time_s) const {
    if (!std::isfinite(time_s) || time_s < 0) {
        throw std::invalid_argument("geometric wavefront time must be finite and nonnegative");
    }
    WavefrontFrame frame;
    if (time_s > duration_s()) {
        return frame;
    }
    const double traveled_m = sound_speed_m_s * time_s;
    std::vector<sonar::Vec3> unfolded;
    unfolded.reserve(rays.size());
    frame.points.reserve(rays.size());
    frame.links.reserve(2 * rays.size());
    for (const auto &ray : rays) {
        const auto position = origin_m + ray.direction * traveled_m;
        unfolded.push_back(position);
        frame.points.push_back({ray.ray_id, ray.ray_id / vertical_rays, ray.ray_id % vertical_rays,
                                crossings(position, dimensions_m), fold(position, dimensions_m)});
    }
    for (unsigned horizontal = 0; horizontal < horizontal_rays; ++horizontal) {
        for (unsigned vertical = 0; vertical < vertical_rays; ++vertical) {
            const std::size_t ray = static_cast<std::size_t>(horizontal) * vertical_rays + vertical;
            if (horizontal + 1 < horizontal_rays) {
                append_folded_chord(frame, ray, ray + vertical_rays, unfolded[ray],
                                    unfolded[ray + vertical_rays], dimensions_m);
            }
            if (vertical + 1 < vertical_rays) {
                append_folded_chord(frame, ray, ray + 1, unfolded[ray], unfolded[ray + 1],
                                    dimensions_m);
            }
        }
    }
    return frame;
}

} // namespace viewer
