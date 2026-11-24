#pragma once

#include "sonar/simulator.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace viewer {

struct WavefrontPoint {
    unsigned ray_id{};
    unsigned horizontal_index{};
    unsigned vertical_index{};
    unsigned segment_index{}; // Zero is the incident leg before the first surface contact.
    std::uint8_t target_surface_mask{};
    sonar::Vec3 position_m;
};

struct WavefrontFrame {
    std::vector<WavefrontPoint> points;
    // Point-index pairs. Neighbor links never cross differing reflection histories.
    std::vector<std::pair<std::size_t, std::size_t>> links;
};

struct WavefrontPath {
    unsigned ray_id{};
    // TX and outward contacts only. Echo return/retrace vertices are excluded.
    std::vector<sonar::PathVertex> outward;
};

/**
 * Low-resolution GEOMETRIC wavefront in the Ping360 fan, not acoustic pressure,
 * an omnidirectional wave, interference, diffraction or a wave-equation solution.
 * Cache once for each configuration/angle; sampling only interpolates known paths.
 * Paths terminate at the last contact retained by the core. Free travel beyond
 * that point, diffuse branching, particle-scattered fronts and echo retraces are absent.
 */
struct GeometricWavefront {
    unsigned horizontal_rays{};
    unsigned vertical_rays{};
    double sound_speed_m_s{};
    double max_range_m{};
    std::vector<WavefrontPath> paths;

    /// One-way propagation window Rmax/c seconds, independent of any selected echo.
    double duration_s() const;
    /// Traveled distance = c*time_s. Returns empty after the propagation window.
    /// Throws std::invalid_argument for negative or non-finite time_s.
    WavefrontFrame sample_at_time(double time_s) const;
};

/// Uses the existing core once with a separate coarse 36 x 7 fan by default.
/// Material settings and reflection limits are retained; volume echo generation
/// is omitted because only outward surface polylines define this geometric front.
/// Throws std::invalid_argument for invalid configuration, angle or resolution.
GeometricWavefront make_wavefront(const sonar::Config &config, unsigned angle_grad,
                                  unsigned horizontal_rays = 36, unsigned vertical_rays = 7);

} // namespace viewer
