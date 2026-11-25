#pragma once

#include "sonar/simulator.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace viewer {

using ScanProfiles = std::array<std::vector<sonar::RangeBin>, 400>;
enum class FilterStrength { Relaxed, Balanced, Strict };

struct InferenceOptions {
    // Apparent-range extent c*transmit_duration/2, in metres. Peak clusters use
    // leading edges; this width is retained as conservative range uncertainty.
    double pulse_width_m{0.015};
    double minimum_range_m{0.75};
    double relative_peak_floor{1e-6};
    unsigned minimum_support_angles{18};
    unsigned minimum_consecutive_angles{7};
    unsigned maximum_gap_angles{2};
    double relative_support_fraction{0.75};
    double global_support_fraction{0.25};
    double retention_width_scale{1.25};
    // Changes filtering/evidence settings only; acquisition widths/ranges are preserved.
    void set_strength(FilterStrength strength);
};

enum class PoolInferenceStatus { Insufficient, Partial, SupportedRectangle };

struct EstimatedWallSegment {
    std::array<double, 2> first_xy_m{};
    std::array<double, 2> last_xy_m{};
    unsigned support_angles{};
};

struct EstimatedWall {
    // Sonar-local XY plane: normal_x*x + normal_y*y = offset_m; offset>0.
    double normal_x{};
    double normal_y{};
    double offset_m{};
    std::array<double, 2> first_xy_m{};
    std::array<double, 2> last_xy_m{};
    // Supported disjoint runs; only small configurable heading gaps are bridged.
    // Legacy first/last fields identify the longest supported segment.
    std::vector<EstimatedWallSegment> segments;
    unsigned support_angles{};
    // Connected run length may include the preset's small bridged heading gaps.
    unsigned longest_run_angles{};
    // Residual of observed peak points, NOT calibrated geometric accuracy.
    double rms_residual_m{};
};

struct PoolEstimate {
    PoolInferenceStatus status{PoolInferenceStatus::Insufficient};
    std::vector<EstimatedWall> walls;
    // NaN marks an angle without a supported finite wall segment.
    std::array<double, 400> predicted_range_m{};
    // Masks only measured bins; an enabled bit does not synthesize or rescale energy.
    std::array<std::vector<std::uint8_t>, 400> retained_bins;
    double bin_width_m{};
    double range_uncertainty_m{};
    std::string message;

    bool retains(unsigned angle_grad, std::size_t bin) const;
};

/**
 * Deterministic first-pass rectangular-pool PRIOR, not SLAM or a general multipath
 * classifier. Consumes only 400 stationary signal profiles and acquisition widths.
 * Channel labels are opaque: every decision uses direct+multipath+volume sums.
 * Requires cross-angle line support and consecutive headings; chooses nearer
 * substantially supported orthogonal planes over farther mirrored candidates. Unsupported walls
 * are omitted and sparse/empty evidence can yield Insufficient or Partial results.
 * No pool dimensions, vehicle pose, Echo, path or simulation-truth labels enter.
 * Assumes a level scan plane; yaw/in-plane rotation is supported, arbitrary tilt
 * is not observable from these inputs. Long pulses, vertical fan projection,
 * near walls inside minimum range and coherent rectangular ghosts remain ambiguous.
 * Malformed/non-finite/negative input or invalid options throw std::invalid_argument.
 */
PoolEstimate infer_pool(const ScanProfiles &profiles, const InferenceOptions &options = {});

} // namespace viewer
