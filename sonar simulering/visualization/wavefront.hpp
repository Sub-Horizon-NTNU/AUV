#pragma once

#include "sonar/simulator.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace viewer {

struct WavefrontPoint {
    unsigned ray_id{};
    unsigned horizontal_index{};
    unsigned vertical_index{};
    unsigned segment_index{}; // Number of crossed planes; zero is the incident front.
    sonar::Vec3 position_m;
};

struct WavefrontFrame {
    std::vector<WavefrontPoint> points;
    // Point-index pairs. Neighbor chords split at every unfolded wall crossing,
    // so reflected patches meet at the wall instead of leaving artificial gaps.
    std::vector<std::pair<std::size_t, std::size_t>> links;
};

struct WavefrontRay {
    unsigned ray_id{};
    sonar::Vec3 direction; // Unit launch direction in the pool frame.
};

/**
 * Low-resolution GEOMETRIC wavefront in the Ping360 fan, not acoustic pressure,
 * an omnidirectional wave, interference, diffraction or a wave-equation solution.
 * Cache launch directions once per configuration/angle. Sampling folds straight
 * propagation through mirrored copies of the rectangular pool. Every ray persists
 * until Rmax/c, including travel before/after contacts retained by the echo model.
 * This ideal-mirror geometry deliberately ignores material energy loss and the echo
 * hit budget. Diffuse branches, particle-scattered fronts and echo retraces are absent.
 */
struct GeometricWavefront {
    unsigned horizontal_rays{};
    unsigned vertical_rays{};
    double sound_speed_m_s{};
    double max_range_m{};
    sonar::Vec3 origin_m;
    sonar::Vec3 dimensions_m;
    std::vector<WavefrontRay> rays;

    /// One-way propagation window Rmax/c seconds, independent of any selected echo.
    double duration_s() const;
    /// Traveled distance = c*time_s. Returns empty after the propagation window.
    /// The first rays.size() points are ray samples; subsequent points split wall seams
    /// and carry UINT_MAX for ray/grid indices. Negative/non-finite time throws
    /// std::invalid_argument. A fixed seam-work guard throws std::runtime_error if exceeded.
    WavefrontFrame sample_at_time(double time_s) const;
};

/// Uses an independent 32 x 192 fan by default, with at most 262144 rays.
/// The pose and midpoint beam sampling match the core; no echo simulation is needed.
/// Throws std::invalid_argument for invalid configuration, angle or resolution.
GeometricWavefront make_wavefront(const sonar::Config &config, unsigned angle_grad,
                                  unsigned horizontal_rays = 32, unsigned vertical_rays = 192);

} // namespace viewer
