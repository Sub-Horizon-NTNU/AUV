#include "inference.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace viewer {
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180;
constexpr unsigned maximum_peaks_per_angle = 24;
using Support = std::bitset<400>;

struct Point {
    unsigned angle{};
    double x{};
    double y{};
};

struct PlaneCandidate {
    double offset{};
    unsigned count{};
    unsigned run{};
    bool supported{};
};

struct Orientation {
    double angle_rad{};
    std::array<PlaneCandidate, 4> planes;
    double score{};
};

double total(const sonar::RangeBin &bin) {
    return bin.direct_intensity + bin.multipath_intensity + bin.volume_intensity;
}

unsigned longest_run(const Support &support) {
    unsigned current = 0, longest = 0;
    for (unsigned index = 0; index < 800; ++index) {
        current = support[index % 400] ? std::min(400U, current + 1) : 0;
        longest = std::max(longest, current);
    }
    return longest;
}

Support bridge_gaps(const Support &support, unsigned maximum_gap) {
    Support result = support;
    for (unsigned angle = 0; angle < 400; ++angle) {
        if (!support[angle]) {
            continue;
        }
        for (unsigned gap = 1; gap <= maximum_gap; ++gap) {
            if (support[(angle + gap) % 400]) {
                break;
            }
            if (support[(angle + gap + 1) % 400]) {
                for (unsigned step = 1; step <= gap; ++step) {
                    result.set((angle + step) % 400);
                }
                break;
            }
        }
    }
    return result;
}

double median(std::vector<double> values) {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

void validate_options(const InferenceOptions &options) {
    if (!std::isfinite(options.pulse_width_m) || options.pulse_width_m < 0 ||
        options.pulse_width_m > 5 || !std::isfinite(options.minimum_range_m) ||
        options.minimum_range_m < 0 || !std::isfinite(options.relative_peak_floor) ||
        options.relative_peak_floor <= 0 || options.relative_peak_floor > 0.1 ||
        options.minimum_support_angles < 8 || options.minimum_support_angles > 400 ||
        options.minimum_consecutive_angles < 3 ||
        options.minimum_consecutive_angles > options.minimum_support_angles ||
        options.maximum_gap_angles > 8 || !std::isfinite(options.relative_support_fraction) ||
        options.relative_support_fraction < 0.1 || options.relative_support_fraction > 1 ||
        !std::isfinite(options.global_support_fraction) || options.global_support_fraction < 0 ||
        options.global_support_fraction > 1 || !std::isfinite(options.retention_width_scale) ||
        options.retention_width_scale < 0.25 || options.retention_width_scale > 4) {
        throw std::invalid_argument("invalid rectangular-pool inference options");
    }
}

std::vector<Point> peak_points(const ScanProfiles &profiles, const InferenceOptions &options,
                               double &bin_width, double &range) {
    std::size_t sample_count = 0;
    for (const auto &profile : profiles) {
        if (!profile.empty()) {
            if (profile.size() < 2 || profile.size() > 1200) {
                throw std::invalid_argument(
                    "inference profiles require 2..1200 uniform range bins");
            }
            sample_count = profile.size();
            bin_width = profile[1].center_range_m - profile[0].center_range_m;
            range = profile.back().center_range_m + bin_width / 2;
            break;
        }
    }
    if (!sample_count) {
        return {};
    }
    if (!std::isfinite(bin_width) || bin_width <= 0 || !std::isfinite(range) || range > 1000) {
        throw std::invalid_argument(
            "inference range metadata must be finite, positive and uniform");
    }
    std::vector<Point> points;
    for (unsigned angle = 0; angle < 400; ++angle) {
        const auto &profile = profiles[angle];
        if (profile.empty()) {
            continue;
        }
        if (profile.size() != sample_count) {
            throw std::invalid_argument("inference scan bin counts must agree");
        }
        double peak = 0;
        for (std::size_t index = 0; index < profile.size(); ++index) {
            const auto &bin = profile[index];
            if (!std::isfinite(bin.center_range_m) ||
                std::abs(bin.center_range_m - (static_cast<double>(index) + 0.5) * bin_width) >
                    1e-7 * std::max(1.0, range) ||
                !std::isfinite(bin.direct_intensity) || bin.direct_intensity < 0 ||
                !std::isfinite(bin.multipath_intensity) || bin.multipath_intensity < 0 ||
                !std::isfinite(bin.volume_intensity) || bin.volume_intensity < 0 ||
                !std::isfinite(total(bin))) {
                throw std::invalid_argument(
                    "inference requires finite nonnegative signals and consistent bin centers");
            }
            peak = std::max(peak, total(bin));
        }
        if (peak == 0) {
            continue;
        }
        const double floor = peak * options.relative_peak_floor;
        unsigned peaks = 0;
        for (std::size_t index = 0; index < profile.size() && peaks < maximum_peaks_per_angle;) {
            if (profile[index].center_range_m < options.minimum_range_m ||
                total(profile[index]) <= floor) {
                ++index;
                continue;
            }
            const std::size_t first = index;
            while (index < profile.size() && total(profile[index]) > floor) {
                ++index;
            }
            const double width = static_cast<double>(index - first) * bin_width;
            // Reject broad noise floors. Causal transmitted pulses become one leading-edge
            // observation, rather than shifting a long plateau's center onto a false wall.
            if (width >
                std::max({4 * bin_width, 2 * options.pulse_width_m + 2 * bin_width, range * 0.1})) {
                continue;
            }
            const double r =
                std::max(options.minimum_range_m, profile[first].center_range_m - bin_width / 2);
            const double bearing = angle * 0.9 * radians;
            points.push_back({angle, r * std::cos(bearing), r * std::sin(bearing)});
            ++peaks;
        }
    }
    return points;
}

Orientation evaluate_orientation(const std::vector<Point> &points, double angle_rad, double range,
                                 double tolerance, const InferenceOptions &options) {
    Orientation result;
    result.angle_rad = angle_rad;
    const double nx = std::cos(angle_rad), ny = std::sin(angle_rad);
    const auto bins = static_cast<std::size_t>(std::ceil(2 * range / tolerance)) + 1;
    std::array<std::vector<Support>, 2> histograms;
    for (auto &histogram : histograms) {
        histogram.resize(bins);
    }
    for (const auto &point : points) {
        const double projections[]{nx * point.x + ny * point.y, -ny * point.x + nx * point.y};
        for (unsigned axis = 0; axis < 2; ++axis) {
            const auto center =
                static_cast<int>(std::floor((projections[axis] + range) / tolerance));
            for (int index = std::max(0, center - 1);
                 index <= std::min(static_cast<int>(bins) - 1, center + 1); ++index) {
                const double rho = -range + (index + 0.5) * tolerance;
                if (std::abs(projections[axis] - rho) <= tolerance) {
                    histograms[axis][static_cast<std::size_t>(index)].set(point.angle);
                }
            }
        }
    }
    std::array<std::vector<PlaneCandidate>, 4> supported;
    for (unsigned axis = 0; axis < 2; ++axis) {
        for (std::size_t index = 0; index < bins; ++index) {
            const double rho = -range + (static_cast<double>(index) + 0.5) * tolerance;
            if (std::abs(rho) < tolerance * 2) {
                continue;
            }
            const auto count = static_cast<unsigned>(histograms[axis][index].count());
            if (count < options.minimum_support_angles) {
                continue;
            }
            const auto run =
                longest_run(bridge_gaps(histograms[axis][index], options.maximum_gap_angles));
            if (run < options.minimum_consecutive_angles) {
                continue;
            }
            const unsigned side = axis * 2 + (rho < 0 ? 1U : 0U);
            supported[side].push_back({std::abs(rho), count, run, true});
        }
    }
    double global_best = 0;
    for (const auto &side : supported) {
        for (const auto &candidate : side) {
            global_best = std::max(global_best, candidate.count + 2.0 * candidate.run);
        }
    }
    for (unsigned side = 0; side < 4; ++side) {
        double strongest = 0;
        for (const auto &candidate : supported[side]) {
            strongest = std::max(strongest, candidate.count + 2.0 * candidate.run);
        }
        for (const auto &candidate : supported[side]) {
            // A short accidental inner line cannot displace a much more persistent
            // plane. Among substantial alternatives, proximity wins over brightness.
            const double evidence = candidate.count + 2.0 * candidate.run;
            if (evidence < strongest * options.relative_support_fraction ||
                evidence < global_best * options.global_support_fraction) {
                continue;
            }
            if (!result.planes[side].supported || candidate.offset < result.planes[side].offset) {
                result.planes[side] = candidate;
            }
        }
    }
    for (const auto &plane : result.planes) {
        if (plane.supported) {
            result.score += plane.count + plane.run;
        }
    }
    return result;
}

EstimatedWall refine_wall(const std::vector<Point> &points, double nx, double ny, double candidate,
                          double tolerance, const InferenceOptions &options) {
    EstimatedWall result;
    result.normal_x = nx;
    result.normal_y = ny;
    result.offset_m = candidate;
    std::array<const Point *, 400> selected{};
    for (unsigned iteration = 0; iteration < 2; ++iteration) {
        selected.fill(nullptr);
        for (const auto &point : points) {
            const double projection = nx * point.x + ny * point.y;
            const double residual = std::abs(projection - result.offset_m);
            if (residual > tolerance) {
                continue;
            }
            const auto *previous = selected[point.angle];
            if (!previous ||
                residual < std::abs(nx * previous->x + ny * previous->y - result.offset_m)) {
                selected[point.angle] = &point;
            }
        }
        std::vector<double> offsets;
        for (const auto *point : selected) {
            if (point) {
                offsets.push_back(nx * point->x + ny * point->y);
            }
        }
        if (offsets.empty()) {
            return result;
        }
        result.offset_m = median(std::move(offsets));
    }
    Support support;
    double squared = 0;
    for (const auto *point : selected) {
        if (!point) {
            continue;
        }
        support.set(point->angle);
        const double residual = nx * point->x + ny * point->y - result.offset_m;
        squared += residual * residual;
    }
    result.support_angles = static_cast<unsigned>(support.count());
    const auto connected = bridge_gaps(support, options.maximum_gap_angles);
    result.longest_run_angles = longest_run(connected);
    if (result.support_angles) {
        result.rms_residual_m = std::sqrt(squared / result.support_angles);
        for (unsigned start = 0; start < 400; ++start) {
            if (!connected[start] || (!connected.all() && connected[(start + 399) % 400])) {
                continue;
            }
            double minimum = std::numeric_limits<double>::infinity(), maximum = -minimum;
            unsigned count = 0;
            for (unsigned step = 0; step < 400 && connected[(start + step) % 400]; ++step) {
                const auto *point = selected[(start + step) % 400];
                if (point) {
                    const double tangent = -ny * point->x + nx * point->y;
                    minimum = std::min(minimum, tangent);
                    maximum = std::max(maximum, tangent);
                    ++count;
                }
            }
            if (count >= options.minimum_consecutive_angles) {
                result.segments.push_back(
                    {{nx * result.offset_m - ny * minimum, ny * result.offset_m + nx * minimum},
                     {nx * result.offset_m - ny * maximum, ny * result.offset_m + nx * maximum},
                     count});
            }
            if (connected.all()) {
                break;
            }
        }
        if (!result.segments.empty()) {
            const auto longest = std::max_element(
                result.segments.begin(), result.segments.end(),
                [](const auto &a, const auto &b) { return a.support_angles < b.support_angles; });
            result.first_xy_m = longest->first_xy_m;
            result.last_xy_m = longest->last_xy_m;
        }
    }
    return result;
}

} // namespace

void InferenceOptions::set_strength(FilterStrength strength) {
    switch (strength) {
    case FilterStrength::Relaxed:
        relative_peak_floor = 1e-7;
        minimum_support_angles = 14;
        minimum_consecutive_angles = 5;
        maximum_gap_angles = 4;
        relative_support_fraction = 0.6;
        global_support_fraction = 0.15;
        retention_width_scale = 1.8;
        break;
    case FilterStrength::Balanced:
        relative_peak_floor = 1e-6;
        minimum_support_angles = 18;
        minimum_consecutive_angles = 7;
        maximum_gap_angles = 2;
        relative_support_fraction = 0.75;
        global_support_fraction = 0.25;
        retention_width_scale = 1.25;
        break;
    case FilterStrength::Strict:
        relative_peak_floor = 1e-5;
        minimum_support_angles = 26;
        minimum_consecutive_angles = 10;
        maximum_gap_angles = 0;
        relative_support_fraction = 0.85;
        global_support_fraction = 0.4;
        retention_width_scale = 0.8;
        break;
    default:
        throw std::invalid_argument("unsupported filter strength");
    }
}

bool PoolEstimate::retains(unsigned angle_grad, std::size_t bin) const {
    return angle_grad < retained_bins.size() && bin < retained_bins[angle_grad].size() &&
           retained_bins[angle_grad][bin] != 0;
}

PoolEstimate infer_pool(const ScanProfiles &profiles, const InferenceOptions &options) {
    validate_options(options);
    PoolEstimate result;
    result.predicted_range_m.fill(std::numeric_limits<double>::quiet_NaN());
    for (unsigned angle = 0; angle < 400; ++angle) {
        result.retained_bins[angle].resize(profiles[angle].size());
    }
    double range = 0;
    const auto points = peak_points(profiles, options, result.bin_width_m, range);
    result.range_uncertainty_m = result.bin_width_m + options.pulse_width_m;
    result.message = "Insufficient cross-angle wall evidence; no pool inferred.";
    if (points.size() < options.minimum_support_angles) {
        return result;
    }
    const double tolerance = std::max(0.04, 2 * result.bin_width_m);
    Orientation best;
    for (unsigned degree = 0; degree < 90; ++degree) {
        const auto candidate =
            evaluate_orientation(points, degree * radians, range, tolerance, options);
        if (candidate.score > best.score) {
            best = candidate;
        }
    }
    const double coarse = best.angle_rad;
    for (int quarter = -4; quarter <= 4; ++quarter) {
        const auto candidate =
            evaluate_orientation(points, coarse + quarter * radians / 4, range, tolerance, options);
        if (candidate.score > best.score) {
            best = candidate;
        }
    }
    for (unsigned side = 0; side < 4; ++side) {
        if (!best.planes[side].supported) {
            continue;
        }
        const double direction = best.angle_rad + (side / 2) * pi / 2 + (side % 2) * pi;
        auto wall = refine_wall(points, std::cos(direction), std::sin(direction),
                                best.planes[side].offset, tolerance, options);
        if (wall.support_angles >= options.minimum_support_angles &&
            wall.longest_run_angles >= options.minimum_consecutive_angles &&
            !wall.segments.empty()) {
            result.walls.push_back(wall);
        }
    }
    if (result.walls.empty()) {
        return result;
    }
    result.status = result.walls.size() == 4 ? PoolInferenceStatus::SupportedRectangle
                                             : PoolInferenceStatus::Partial;
    result.message = result.walls.size() == 4
                         ? "Four supported walls under a rectangular, level-scan prior. Heuristic "
                           "estimate, not truth."
                         : "Partial rectangular-pool evidence; unsupported walls omitted.";
    for (unsigned angle = 0; angle < 400; ++angle) {
        const double bearing = angle * 0.9 * radians;
        const double x = std::cos(bearing), y = std::sin(bearing);
        double nearest = std::numeric_limits<double>::infinity(), radial_tolerance = 0;
        for (const auto &wall : result.walls) {
            const double incidence = wall.normal_x * x + wall.normal_y * y;
            if (incidence <= 0.15) {
                continue;
            }
            const double distance = wall.offset_m / incidence;
            if (distance < options.minimum_range_m || distance > range || distance >= nearest) {
                continue;
            }
            const double tangent = -wall.normal_y * x * distance + wall.normal_x * y * distance;
            const auto coordinate = [&](const std::array<double, 2> &p) {
                return -wall.normal_y * p[0] + wall.normal_x * p[1];
            };
            const bool supported_segment =
                std::any_of(wall.segments.begin(), wall.segments.end(), [&](const auto &segment) {
                    return tangent >= coordinate(segment.first_xy_m) - tolerance &&
                           tangent <= coordinate(segment.last_xy_m) + tolerance;
                });
            if (!supported_segment) {
                continue;
            }
            nearest = distance;
            radial_tolerance = options.retention_width_scale *
                               ((tolerance + wall.rms_residual_m * 2) / std::max(0.25, incidence) +
                                result.bin_width_m);
        }
        if (!std::isfinite(nearest)) {
            continue;
        }
        result.predicted_range_m[angle] = nearest;
        for (std::size_t bin = 0; bin < profiles[angle].size(); ++bin) {
            const auto &sample = profiles[angle][bin];
            if (total(sample) > 0 && sample.center_range_m >= nearest - radial_tolerance &&
                sample.center_range_m <= nearest + options.pulse_width_m + radial_tolerance) {
                result.retained_bins[angle][bin] = 1;
            }
        }
    }
    return result;
}

} // namespace viewer
