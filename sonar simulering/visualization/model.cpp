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
        std::array<std::vector<sonar::RangeBin>, 400> profiles;
        double reference = 0;
        for (unsigned heading = 0; heading < 400; ++heading) {
            if (heading == angle) {
                profiles[heading] = result.profile;
            } else {
                profiles[heading] = sonar::simulate_ping(candidate, heading).profile;
            }
            for (const auto &bin : profiles[heading]) {
                reference = std::max(reference, bin.direct_intensity + bin.multipath_intensity +
                                                    bin.volume_intensity);
            }
        }
        auto cloud = sonar::volume_points(candidate);
        auto front = make_wavefront(candidate, angle);
        config = candidate;
        draft = candidate;
        ping = std::move(result);
        scan = std::move(profiles);
        scan_reference = reference;
        particles = std::move(cloud);
        wavefront = std::move(front);
        ++scan_revision;
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
        if (!ping.echoes.empty() && !accepts(ping.echoes[selected])) {
            selected = ping.echoes.size();
            for (std::size_t index = 0; index < ping.echoes.size(); ++index) {
                if (accepts(ping.echoes[index])) {
                    selected = index;
                    break;
                }
            }
        }
        progress = 0.0;
        playing = true;
        error = false;
        status = "400-angle stationary scan cached. Surface and particle models are uncalibrated "
                 "approximations.";
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
    set_angle(angle);
}

void Model::set_angle(unsigned angle) {
    auto result = sonar::simulate_ping(config, angle);
    auto front = make_wavefront(config, angle);
    wavefront = std::move(front);
    progress = 0;
    ping = std::move(result);
    selected = ping.echoes.size();
    for (std::size_t index = 0; index < ping.echoes.size(); ++index) {
        if (accepts(ping.echoes[index])) {
            select(index);
            break;
        }
    }
}

void Model::set_channel(Channel new_channel) {
    const auto *previous = echo();
    const double range_m = previous ? previous->apparent_range_m : 0.0;
    channel = new_channel;
    select_range(range_m);
    if (echo() && !error) {
        status = "Selected the nearest eligible arrival in this channel.";
    }
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
        int index = selected < ping.echoes.size() ? static_cast<int>(selected) : 0;
        for (int step = 0; step < count; ++step) {
            index = (index + delta + count) % count;
            if (accepts(ping.echoes[static_cast<std::size_t>(index)])) {
                select(static_cast<std::size_t>(index));
                return;
            }
        }
    }
}

void Model::select_range(double range_m) {
    double nearest = std::numeric_limits<double>::infinity();
    std::size_t index = ping.echoes.size();
    for (std::size_t candidate = 0; candidate < ping.echoes.size(); ++candidate) {
        const auto &arrival = ping.echoes[candidate];
        if (!arrival.in_receive_window || arrival.relative_intensity <= 0.0 || !accepts(arrival)) {
            continue;
        }
        const double distance = std::abs(arrival.apparent_range_m - range_m);
        if (distance < nearest) {
            nearest = distance;
            index = candidate;
        }
    }
    if (index < ping.echoes.size()) {
        select(index);
    } else {
        selected = ping.echoes.size();
        status = "No nonzero arrival in this channel at the selected angle.";
    }
}

bool Model::accepts(const sonar::Echo &arrival) const {
    return channel == Channel::Combined ||
           (channel == Channel::Volume && arrival.kind == sonar::EchoKind::Volume) ||
           (channel == Channel::Multipath && arrival.is_multipath()) ||
           (channel == Channel::Direct && arrival.kind == sonar::EchoKind::Surface &&
            !arrival.is_multipath());
}

double Model::channel_energy(const sonar::RangeBin &bin) const {
    switch (channel) {
    case Channel::Direct:
        return bin.direct_intensity;
    case Channel::Multipath:
        return bin.multipath_intensity;
    case Channel::Volume:
        return bin.volume_intensity;
    case Channel::Combined:
        return bin.direct_intensity + bin.multipath_intensity + bin.volume_intensity;
    }
    return 0;
}

double Model::display_level(double energy) const {
    if (energy <= 0 || scan_reference <= 0) {
        return 0;
    }
    return std::clamp((10 * std::log10(energy / scan_reference) + gain_db + dynamic_range_db) /
                          dynamic_range_db,
                      0.0, 1.0);
}

bool Model::select_image(double local_x, double local_y) {
    const double radius = std::hypot(local_x, local_y);
    if (!std::isfinite(radius) || radius > 1) {
        return false;
    }
    const double degrees = std::atan2(local_y, local_x) * 180 / 3.14159265358979323846;
    const auto angle =
        static_cast<unsigned>(std::lround((degrees < 0 ? degrees + 360 : degrees) / 0.9)) % 400;
    set_angle(angle);
    select_range(radius * config.settings.max_range_m);
    return true;
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
    if (echo.kind == sonar::EchoKind::Volume) {
        return "particle " + std::to_string(echo.scatterer_id) + " (single scattering)";
    }
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
