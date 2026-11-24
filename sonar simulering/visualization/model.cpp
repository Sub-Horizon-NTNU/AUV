#include "model.hpp"

#include "sonar/io.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

namespace viewer {

Model::Model(std::filesystem::path path) : config_path(std::move(path)) {
    if (!config_path.empty()) {
        config = sonar::read_config(config_path);
    }
    draft = config;
    apply(config, 0);
}

bool Model::apply(const sonar::Config &candidate, unsigned angle) {
    try {
        auto result = sonar::simulate_ping(candidate, angle);
        config = candidate;
        draft = candidate;
        ping = std::move(result);
        selected = 0;
        const unsigned center_ray =
            (config.settings.horizontal_rays / 2) * config.settings.vertical_rays +
            config.settings.vertical_rays / 2;
        for (std::size_t index = 0; index < ping.echoes.size(); ++index) {
            if (ping.echoes[index].ray_id == center_ray && ping.echoes[index].is_multipath()) {
                selected = index;
                break;
            }
        }
        progress = 0.0;
        playing = true;
        error = false;
        status = "Simulation updated. Geometry and echo labels are model predictions.";
        return true;
    } catch (const std::exception &failure) {
        error = true;
        status = std::string("Not applied: ") + failure.what();
        return false;
    }
}

void Model::reload() {
    try {
        const auto candidate =
            config_path.empty() ? sonar::Config{} : sonar::read_config(config_path);
        apply(candidate, ping.angle_grad);
    } catch (const std::exception &failure) {
        error = true;
        status = std::string("Reload failed: ") + failure.what();
    }
}

void Model::change_angle(int delta) {
    const auto angle =
        static_cast<unsigned>((static_cast<int>(ping.angle_grad) + delta + 400) % 400);
    apply(config, angle);
}

void Model::select(std::size_t index) {
    if (index < ping.echoes.size()) {
        selected = index;
        progress = 0.0;
        playing = true;
    }
}

void Model::cycle_echo(int delta) {
    if (!ping.echoes.empty()) {
        const auto count = static_cast<int>(ping.echoes.size());
        select(static_cast<std::size_t>((static_cast<int>(selected) + delta + count) % count));
    }
}

void Model::select_range(double range_m) {
    double nearest = std::numeric_limits<double>::infinity();
    std::size_t index = selected;
    for (std::size_t candidate = 0; candidate < ping.echoes.size(); ++candidate) {
        const auto &arrival = ping.echoes[candidate];
        if (!arrival.in_receive_window || arrival.relative_intensity <= 0.0) {
            continue;
        }
        const double distance = std::abs(arrival.apparent_range_m - range_m);
        if (distance < nearest) {
            nearest = distance;
            index = candidate;
        }
    }
    select(index);
}

void Model::tick(double seconds) {
    if (playing) {
        progress = std::min(1.0, progress + seconds / 5.0);
        if (progress == 1.0) {
            playing = false;
        }
    }
}

const sonar::Echo *Model::echo() const {
    return selected < ping.echoes.size() ? &ping.echoes[selected] : nullptr;
}

sonar::Vec3 path_position(const sonar::Echo &echo, double fraction) {
    double remaining = echo.total_path_m * std::clamp(fraction, 0.0, 1.0);
    for (std::size_t index = 1; index < echo.path.size(); ++index) {
        const auto start = echo.path[index - 1].position_m;
        const auto delta = echo.path[index].position_m - start;
        const double segment = sonar::length(delta);
        if (remaining <= segment && segment > 0.0) {
            return start + delta * (remaining / segment);
        }
        remaining -= segment;
    }
    return echo.path.back().position_m;
}

sonar::Vec3 apparent_position(const sonar::Echo &echo) {
    const auto start = echo.path.front().position_m;
    const auto delta = echo.path[1].position_m - start;
    return start + delta * (echo.apparent_range_m / sonar::length(delta));
}

std::string contacts(const sonar::Echo &echo) {
    std::string result;
    for (std::size_t vertex = 1; vertex <= echo.outward_hits; ++vertex) {
        if (!result.empty()) {
            result += " > ";
        }
        bool first = true;
        for (unsigned face = 0; face < 6; ++face) {
            if ((echo.path[vertex].surface_mask & (1U << face)) != 0) {
                if (!first) {
                    result += '+';
                }
                result += sonar::surface_name(static_cast<sonar::Surface>(face));
                first = false;
            }
        }
    }
    return result;
}

} // namespace viewer
