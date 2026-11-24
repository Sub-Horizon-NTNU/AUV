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
    GeometricWavefront wavefront;
    std::filesystem::path config_path;
    std::size_t selected{};
    double progress{};
    bool playing{true};
    Projection projection{Projection::Perspective};
    float orbit_deg{35.0F};
    std::string status{"Illustrative pool and acoustic coefficients. No measured sonar data."};
    bool error{};

    explicit Model(std::filesystem::path path);
    bool apply(const sonar::Config &candidate, unsigned angle);
    void reload();
    void change_angle(int delta);
    void set_angle(unsigned angle);
    void set_channel(Channel new_channel);
    void select(std::size_t index);
    void cycle_echo(int delta);
    void select_range(double range_m);
    void tick(double seconds);
    const sonar::Echo *echo() const;
    bool accepts(const sonar::Echo &arrival) const;
    double channel_energy(const sonar::RangeBin &bin) const;
    double display_level(double energy) const;
    // Local plan view: +x forward/up, +y left; normalized radius [0,1].
    bool select_image(double local_x, double local_y);
};

sonar::Vec3 path_position(const sonar::Echo &echo, double fraction);
// Uses the existing pool-frame first segment, preserving the core pose transform.
sonar::Vec3 apparent_position(const sonar::Echo &echo);
std::string contacts(const sonar::Echo &echo);

} // namespace viewer
