#include "wavefront.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace viewer {

GeometricWavefront make_wavefront(const sonar::Config &config, unsigned angle_grad,
                                  unsigned horizontal_rays, unsigned vertical_rays) {
    sonar::validate(config);
    auto coarse = config;
    coarse.settings.horizontal_rays = horizontal_rays;
    coarse.settings.vertical_rays = vertical_rays;
    coarse.settings.volume_scatterers = 0;
    const auto ping = sonar::simulate_ping(coarse, angle_grad);
    const std::size_t count = static_cast<std::size_t>(horizontal_rays) * vertical_rays;
    std::vector<const sonar::Echo *> longest(count, nullptr);
    for (const auto &echo : ping.echoes) {
        if (echo.kind == sonar::EchoKind::Surface &&
            (!longest[echo.ray_id] || echo.outward_hits > longest[echo.ray_id]->outward_hits)) {
            longest[echo.ray_id] = &echo;
        }
    }
    GeometricWavefront result{horizontal_rays,
                              vertical_rays,
                              config.settings.sound_speed_m_s,
                              config.settings.max_range_m,
                              {}};
    result.paths.reserve(count);
    for (const auto *echo : longest) {
        if (echo) {
            const auto end =
                echo->path.begin() + static_cast<std::ptrdiff_t>(echo->outward_hits + 1);
            result.paths.push_back({echo->ray_id, {echo->path.begin(), end}});
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
    WavefrontFrame result;
    if (time_s > duration_s()) {
        return result;
    }
    const double traveled_m = sound_speed_m_s * time_s;
    const auto missing = std::numeric_limits<std::size_t>::max();
    const std::size_t ray_count = static_cast<std::size_t>(horizontal_rays) * vertical_rays;
    std::vector<std::size_t> point_by_ray(ray_count, missing);
    std::vector<std::size_t> path_by_ray(ray_count, missing);
    result.points.reserve(paths.size());
    for (std::size_t path_index = 0; path_index < paths.size(); ++path_index) {
        const auto &path = paths[path_index];
        double remaining = traveled_m;
        for (std::size_t vertex = 1; vertex < path.outward.size(); ++vertex) {
            const auto start = path.outward[vertex - 1].position_m;
            const auto delta = path.outward[vertex].position_m - start;
            const double segment_m = sonar::length(delta);
            if (segment_m <= 0) {
                continue;
            }
            if (remaining <= segment_m) {
                point_by_ray[path.ray_id] = result.points.size();
                path_by_ray[path.ray_id] = path_index;
                result.points.push_back(
                    {path.ray_id, path.ray_id / vertical_rays, path.ray_id % vertical_rays,
                     static_cast<unsigned>(vertex - 1), path.outward[vertex].surface_mask,
                     start + delta * (remaining / segment_m)});
                break;
            }
            remaining -= segment_m;
        }
    }
    const auto link = [&](std::size_t first_ray, std::size_t second_ray) {
        const auto first_index = point_by_ray[first_ray];
        const auto second_index = point_by_ray[second_ray];
        if (first_index == missing || second_index == missing) {
            return;
        }
        const auto &first = result.points[first_index];
        const auto &second = result.points[second_index];
        if (first.segment_index != second.segment_index) {
            return;
        }
        const auto &first_path = paths[path_by_ray[first_ray]].outward;
        const auto &second_path = paths[path_by_ray[second_ray]].outward;
        // Include the current target face as well as earlier contacts: a line is
        // never stretched between branches on different sides of a reflecting edge.
        for (unsigned vertex = 1; vertex <= first.segment_index + 1; ++vertex) {
            if (first_path[vertex].surface_mask != second_path[vertex].surface_mask) {
                return;
            }
        }
        result.links.emplace_back(first_index, second_index);
    };
    for (unsigned horizontal = 0; horizontal < horizontal_rays; ++horizontal) {
        for (unsigned vertical = 0; vertical < vertical_rays; ++vertical) {
            const std::size_t ray = static_cast<std::size_t>(horizontal) * vertical_rays + vertical;
            if (horizontal + 1 < horizontal_rays) {
                link(ray, ray + vertical_rays);
            }
            if (vertical + 1 < vertical_rays) {
                link(ray, ray + 1);
            }
        }
    }
    return result;
}

} // namespace viewer
