#pragma once

#include "sonar/simulator.hpp"
#include "wavefront.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace viewer {

enum class Projection { Perspective, Top, Side };
enum class Channel { Combined, Direct, Multipath, Volume };

// Owns a selected ping and all 400 stationary scan profiles. Display changes reuse the scan.
struct Model {
    sonar::Config config;
    sonar::Config draft;
    sonar::Ping ping;
    std::array<std::vector<sonar::RangeBin>, 400> scan;
    std::vector<sonar::Vec3> particles;
    double scan_reference{}; // Strongest combined range bin over all 400 stationary angles.
    unsigned scan_revision{};
    Channel channel{Channel::Combined};
    double gain_db{};
    double dynamic_range_db{80};
    bool scattering_controls{};
    bool wavefront_mode{};
    bool all_returns{true};
    bool move_sonar_mode{};
    bool position_dragging{};
    sonar::Vec3 position_preview;
    GeometricWavefront wavefront;
    std::filesystem::path config_path;
    std::size_t selected{};
    double progress{};
    bool playing{true};
    Projection projection{Projection::Perspective};
    bool sonar_aligned{};
    float orbit_deg{35.0F};
    float camera_pitch_deg{32.0F};
    float camera_zoom{1.0F};
    bool show_particle_markers{true};
    bool help_open{};
    bool camera_dragging{};
    bool settings_open{};
    bool details_open{};
    bool profile_open{};
    bool scan_playing{};
    double scan_phase_grad{};
    double scan_speed_grad_s{40.0};
    std::string status{"Illustrative pool and acoustic coefficients. No measured sonar data."};
    bool error{};

    explicit Model(std::filesystem::path path);
    bool apply(const sonar::Config &candidate, unsigned angle);
    bool move_stationary(sonar::Vec3 position);
    void reload();
    void change_angle(int delta);
    void set_angle(unsigned angle, bool manual = true);
    void advance_scan(double seconds);
    void set_scan_playing(bool enabled);
    void set_channel(Channel new_channel);
    void select(std::size_t index);
    void cycle_echo(int delta);
    void select_range(double range_m);
    void tick(double seconds);
    const sonar::Echo *echo() const;
    bool accepts(const sonar::Echo &arrival) const;
    bool received_route(const sonar::Echo &arrival) const;
    double playback_time_s() const;
    double route_fraction(const sonar::Echo &arrival) const;
    std::size_t arrived_count() const;
    std::size_t received_count() const;
    double channel_energy(const sonar::RangeBin &bin) const;
    double display_level(double energy) const;
    // Local plan view: +x forward/up, +y left; normalized radius [0,1].
    bool select_image(double local_x, double local_y);
    void orbit_camera(float yaw_delta_deg, float pitch_delta_deg);
    void zoom_camera(float wheel_delta);
    void reset_camera();
    void align_with_sonar();
    sonar::Vec3 sonar_to_pool_direction(sonar::Vec3 local) const;
    sonar::Vec3 pool_to_sonar(sonar::Vec3 position) const;
};

double return_fraction(const sonar::Echo &echo);
sonar::Vec3 path_position(const sonar::Echo &echo, double fraction);
// Uses the existing pool-frame first segment, preserving the core pose transform.
sonar::Vec3 apparent_position(const sonar::Echo &echo);
std::string contacts(const sonar::Echo &echo);

} // namespace viewer
