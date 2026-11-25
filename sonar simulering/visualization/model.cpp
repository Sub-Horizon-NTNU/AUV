#include "model.hpp"

#include "sonar/io.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

namespace viewer {

void Model::orbit_camera(float yaw_delta_deg, float pitch_delta_deg) {
    orbit_deg = std::fmod(orbit_deg + yaw_delta_deg, 360.0F);
    camera_pitch_deg = std::clamp(camera_pitch_deg + pitch_delta_deg, -80.0F, 80.0F);
}

void Model::zoom_camera(float wheel_delta) {
    camera_zoom = std::clamp(camera_zoom * std::exp(-wheel_delta * 0.12F), 0.3F, 3.5F);
}

void Model::reset_camera() {
    orbit_deg = 35.0F;
    camera_pitch_deg = 32.0F;
    camera_zoom = 1.0F;
}

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
        const InferenceOptions inference_options{
            candidate.settings.sound_speed_m_s * candidate.settings.transmit_duration_us * 1e-6 / 2,
            sonar::minimum_range_m};
        auto fitted = infer_pool(profiles, inference_options);
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
        estimate = std::move(fitted);
        estimate_revision = scan_revision;
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
        restart_playback();
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

bool Model::move_stationary(sonar::Vec3 position) {
    position.z = config.pose.position_m.z;
    const double clearance = config.settings.receiver_radius_m;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        position.x <= clearance || position.x >= config.pool.length_m - clearance ||
        position.y <= clearance || position.y >= config.pool.width_m - clearance ||
        position.z <= clearance || position.z >= config.pool.depth_m - clearance) {
        error = true;
        status = "Position not applied: keep the receiver aperture clear of all pool surfaces.";
        return false;
    }
    auto candidate = config;
    candidate.pose.position_m = position;
    const bool applied = apply(candidate, ping.angle_grad);
    if (applied) {
        set_scan_playing(false);
        status =
            "Stationary sonar moved; all 400 angles recalculated. No vehicle motion simulated.";
    }
    return applied;
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

void Model::set_angle(unsigned angle, bool manual) {
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
    if (manual) {
        restart_playback();
        scan_playing = false;
        scan_phase_grad = angle;
    }
}

void Model::set_scan_playing(bool enabled) {
    if (enabled && !scan_playing) {
        scan_phase_grad = ping.angle_grad;
        if (wavefront_mode && !emission_started) {
            emitted_fronts.push_back({make_wavefront(config, ping.angle_grad, 8, 64),
                                      ping.angle_grad, scan_revision,
                                      progress * display_horizon_s()});
            emission_started = true;
            emission_phase_s = 0;
        }
    }
    scan_playing = enabled;
}

void Model::advance_scan(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0) {
        throw std::invalid_argument("scan time step must be finite and nonnegative");
    }
    if (!scan_playing) {
        return;
    }
    scan_speed_grad_s = std::clamp(scan_speed_grad_s, 2.0, 80.0);
    scan_phase_grad = std::fmod(
        scan_phase_grad + std::fmod(seconds, 400.0 / scan_speed_grad_s) * scan_speed_grad_s, 400.0);
    const auto angle = static_cast<unsigned>(scan_phase_grad);
    if (angle != ping.angle_grad) {
        const double saved_progress = progress;
        const bool saved_playing = playing;
        set_angle(angle, false);
        progress = saved_progress;
        playing = saved_playing;
    }
}

void Model::set_channel(Channel new_channel) {
    estimated_pool = false;
    const auto *previous = echo();
    const double range_m = previous ? previous->apparent_range_m : 0.0;
    channel = new_channel;
    select_range(range_m);
    if (echo() && !error) {
        status = "Selected the nearest eligible arrival in this channel.";
    }
}

void Model::set_estimated_pool(bool enabled) {
    if (enabled) {
        set_channel(Channel::Combined);
    }
    estimated_pool = enabled;
    if (enabled) {
        status = "Estimated pool: level-scan rectangular prior; support/residual are heuristics, "
                 "not probabilities.";
    }
}

double Model::image_energy(unsigned heading, std::size_t bin) const {
    const auto &sample = scan[heading][bin];
    if (!estimated_pool || estimate.status == PoolInferenceStatus::Insufficient) {
        return channel_energy(sample);
    }
    return estimate.retains(heading, bin)
               ? sample.direct_intensity + sample.multipath_intensity + sample.volume_intensity
               : 0;
}

void Model::select(std::size_t index) {
    if (index < ping.echoes.size()) {
        selected = index;
        if (!all_returns && !wavefront_mode) {
            progress = 0.0;
            playing = true;
        }
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

bool Model::received_route(const sonar::Echo &arrival) const {
    return accepts(arrival) && arrival.in_receive_window && arrival.relative_intensity > 0 &&
           arrival.path.size() >= 2;
}

double Model::display_horizon_s() const {
    if (simulation_window_ms > 0) {
        return simulation_window_ms / 1000;
    }
    if (wavefront_mode) {
        return wavefront.duration_s();
    }
    if (all_returns) {
        return 2 * config.settings.max_range_m / config.settings.sound_speed_m_s;
    }
    const auto *selected_echo = echo();
    return selected_echo ? selected_echo->travel_time_s
                         : 2 * config.settings.max_range_m / config.settings.sound_speed_m_s;
}

double Model::playback_time_s() const {
    return progress * display_horizon_s();
}

double Model::emission_rate_hz() const {
    return std::min(4.0, 31.0 / playback_seconds);
}

void Model::restart_playback() {
    progress = 0;
    playing = true;
    emitted_fronts.clear();
    emission_phase_s = 0;
    emission_started = false;
}

double Model::route_fraction(const sonar::Echo &arrival) const {
    return arrival.travel_time_s > 0
               ? std::clamp(playback_time_s() / arrival.travel_time_s, 0.0, 1.0)
               : 1;
}

std::size_t Model::arrived_count() const {
    return static_cast<std::size_t>(
        std::count_if(ping.echoes.begin(), ping.echoes.end(), [&](const sonar::Echo &arrival) {
            return received_route(arrival) && playback_time_s() >= arrival.travel_time_s;
        }));
}

std::size_t Model::received_count() const {
    return static_cast<std::size_t>(
        std::count_if(ping.echoes.begin(), ping.echoes.end(),
                      [&](const sonar::Echo &arrival) { return received_route(arrival); }));
}

double route_transport_energy(const sonar::Config &config, const sonar::Echo &echo,
                              double fraction) {
    const double distance = echo.total_path_m * std::clamp(fraction, 0.0, 1.0);
    double energy = std::exp(-config.settings.absorption_per_m * distance) /
                    std::pow(std::max(1.0, distance), 2);
    double traveled = 0;
    for (std::size_t index = 1; index + 1 < echo.path.size(); ++index) {
        traveled += sonar::length(echo.path[index].position_m - echo.path[index - 1].position_m);
        if (traveled > distance) {
            break;
        }
        const auto &contact = echo.path[index];
        if (contact.volume_scatter) {
            energy *= config.settings.volume_strength;
            continue;
        }
        const auto mask = contact.surface_mask;
        for (unsigned face = 0; face < 6; ++face) {
            if (!(mask & (1U << face))) {
                continue;
            }
            energy *= face == 4   ? config.pool.floor_reflection
                      : face == 5 ? config.pool.surface_reflection
                                  : config.pool.wall_reflection;
        }
    }
    return energy; // R-only contact proxy; does not replace calibrated scattering or receiver
                   // energy.
}

double return_fraction(const sonar::Echo &echo) {
    return echo.total_path_m > 0 ? std::clamp(echo.outbound_path_m / echo.total_path_m, 0.0, 1.0)
                                 : 0.5;
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
    if (estimated_pool) {
        const auto bin =
            std::min(scan[angle].size() - 1,
                     static_cast<std::size_t>(radius * static_cast<double>(scan[angle].size())));
        if (estimate.status != PoolInferenceStatus::Insufficient && !estimate.retains(angle, bin)) {
            status = "No retained observed return here; the estimate assumes stationary "
                     "rectangular walls.";
            return false;
        }
        set_angle(angle);
        return true;
    }
    set_angle(angle);
    select_range(radius * config.settings.max_range_m);
    return true;
}

sonar::Vec3 Model::sonar_to_pool_direction(sonar::Vec3 value) const {
    constexpr double radians = 3.14159265358979323846 / 180.0;
    const double roll = config.pose.roll_deg * radians;
    const double pitch = config.pose.pitch_deg * radians;
    const double yaw = config.pose.yaw_deg * radians;
    const sonar::Vec3 rolled{value.x, std::cos(roll) * value.y - std::sin(roll) * value.z,
                             std::sin(roll) * value.y + std::cos(roll) * value.z};
    const sonar::Vec3 pitched{std::cos(pitch) * rolled.x + std::sin(pitch) * rolled.z, rolled.y,
                              -std::sin(pitch) * rolled.x + std::cos(pitch) * rolled.z};
    return {std::cos(yaw) * pitched.x - std::sin(yaw) * pitched.y,
            std::sin(yaw) * pitched.x + std::cos(yaw) * pitched.y, pitched.z};
}

sonar::Vec3 Model::pool_to_sonar(sonar::Vec3 position) const {
    const auto delta = position - config.pose.position_m;
    return {sonar::dot(delta, sonar_to_pool_direction({1, 0, 0})),
            sonar::dot(delta, sonar_to_pool_direction({0, 1, 0})),
            sonar::dot(delta, sonar_to_pool_direction({0, 0, 1}))};
}

void Model::align_with_sonar() {
    sonar_aligned = true;
    projection = Projection::Top;
    camera_dragging = false;
}

void Model::tick(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0) {
        throw std::invalid_argument("playback step must be finite and nonnegative");
    }
    const double starting_angle = scan_playing ? scan_phase_grad : ping.angle_grad;
    const double horizon = display_horizon_s();
    if (playing && wavefront_mode && (scan_playing || emission_started)) {
        // Immutable emission snapshots retain their launch pose and angle as the head moves.
        for (auto &front : emitted_fronts) {
            front.age_s += seconds * horizon / playback_seconds;
        }
        emitted_fronts.erase(std::remove_if(emitted_fronts.begin(), emitted_fronts.end(),
                                            [&](const EmittedFront &front) {
                                                return front.age_s > horizon + 1e-12;
                                            }),
                             emitted_fronts.end());
        const auto emit = [&](double at) {
            const double age = (seconds - at) * horizon / playback_seconds;
            if (age > horizon + 1e-12 || emitted_fronts.size() >= 32) {
                return;
            }
            const double angle =
                std::fmod(starting_angle + (scan_playing ? scan_speed_grad_s * at : 0), 400.0);
            const auto heading = static_cast<unsigned>(std::floor(angle + 1e-9)) % 400;
            emitted_fronts.push_back(
                {make_wavefront(config, heading, 8, 64), heading, scan_revision, age});
        };
        if (!emission_started) {
            emit(0);
            emission_started = true;
        }
        const double interval = 1.0 / emission_rate_hz();
        const double first = interval - emission_phase_s;
        if (seconds + 1e-12 >= first) {
            const double count = std::floor((seconds - first + 1e-12) / interval) + 1;
            const unsigned retained = static_cast<unsigned>(std::min(32.0, count));
            for (unsigned sample = 0; sample < retained; ++sample) {
                emit(first + (count - retained + sample) * interval);
            }
        }
        emission_phase_s = std::fmod(emission_phase_s + seconds, interval);
        if (interval - emission_phase_s < 1e-10) {
            emission_phase_s = 0;
        }
    }
    advance_scan(seconds);
    if (playing) {
        const double advanced = progress + seconds / playback_seconds;
        if (loop_playback) {
            progress = std::fmod(advanced, 1.0);
        } else {
            progress = std::min(1.0, advanced);
            if (progress == 1.0) {
                playing = false;
            }
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
    for (std::size_t vertex = 1; vertex + 1 < echo.path.size(); ++vertex) {
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
