#pragma once

#include "sonar/simulator.hpp"

#include <filesystem>
#include <string>

namespace viewer {

enum class Projection { Perspective, Top, Side };

// Owns one cached stationary ping. Camera and playback changes do not resimulate it.
struct Model {
    sonar::Config config;
    sonar::Config draft;
    sonar::Ping ping;
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
    void select(std::size_t index);
    void cycle_echo(int delta);
    void select_range(double range_m);
    void tick(double seconds);
    const sonar::Echo *echo() const;
};

sonar::Vec3 path_position(const sonar::Echo &echo, double fraction);
// Uses the existing pool-frame first segment, preserving the core pose transform.
sonar::Vec3 apparent_position(const sonar::Echo &echo);
std::string contacts(const sonar::Echo &echo);

} // namespace viewer
