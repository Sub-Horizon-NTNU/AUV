#include "inference.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double bin_width = 0.025;
constexpr unsigned bins = 1200;

void check(bool condition, const char *message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

viewer::ScanProfiles empty_scan() {
    viewer::ScanProfiles result;
    for (auto &profile : result) {
        profile.resize(bins);
        for (unsigned bin = 0; bin < bins; ++bin) {
            profile[bin].center_range_m = (bin + 0.5) * bin_width;
        }
    }
    return result;
}

void pulse(std::vector<sonar::RangeBin> &profile, double arrival, double width, double energy,
           unsigned channel) {
    for (unsigned index = 0; index < profile.size(); ++index) {
        const double begin = index * bin_width, end = begin + bin_width;
        const double overlap =
            std::max(0.0, std::min(end, arrival + width) - std::max(begin, arrival));
        const double value = overlap * energy / width;
        if (channel == 0) {
            profile[index].direct_intensity += value;
        } else if (channel == 1) {
            profile[index].multipath_intensity += value;
        } else {
            profile[index].volume_intensity += value;
        }
    }
}

struct RectangleFixture {
    double rotation_deg{17};
    std::array<double, 4> distances{5, 3, 4, 2};
    std::array<double, 2> normal(unsigned side) const {
        const double angle = rotation_deg * pi / 180 + (side / 2) * pi / 2 + (side % 2) * pi;
        return {std::cos(angle), std::sin(angle)};
    }
    std::pair<double, unsigned> arrival(unsigned angle) const {
        const double bearing = angle * 0.9 * pi / 180;
        double nearest = 1000;
        unsigned wall = 0;
        for (unsigned side = 0; side < 4; ++side) {
            const auto n = normal(side);
            const double cosine = n[0] * std::cos(bearing) + n[1] * std::sin(bearing);
            if (cosine > 0 && distances[side] / cosine < nearest) {
                nearest = distances[side] / cosine;
                wall = side;
            }
        }
        return {nearest, wall};
    }
};

viewer::ScanProfiles synthetic(const RectangleFixture &fixture, double width = 0.03,
                               bool ghosts = true, bool particles = true, int missing_wall = -1) {
    auto result = empty_scan();
    std::mt19937 random(123);
    for (unsigned angle = 0; angle < 400; ++angle) {
        const auto arrival = fixture.arrival(angle);
        if (static_cast<int>(arrival.second) != missing_wall) {
            pulse(result[angle], arrival.first, width, 1, 0);
            if (ghosts) {
                pulse(result[angle], 2 * arrival.first, width, 1000, 1);
                pulse(result[angle], 3 * arrival.first, width, 100, 1);
            }
        }
        if (particles) {
            for (unsigned point = 0; point < 2; ++point) {
                const double range = 0.9 + (random() % 20000) * 0.001;
                pulse(result[angle], range, 0.03, 10, 2);
            }
        }
    }
    return result;
}

void check_fixture(const viewer::PoolEstimate &estimate, const RectangleFixture &fixture,
                   double tolerance) {
    check(estimate.status == viewer::PoolInferenceStatus::SupportedRectangle &&
              estimate.walls.size() == 4,
          "four cross-angle supported walls should be inferred from the rectangular fixture");
    for (unsigned side = 0; side < 4; ++side) {
        const auto truth = fixture.normal(side);
        const auto wall = std::max_element(
            estimate.walls.begin(), estimate.walls.end(), [&](const auto &a, const auto &b) {
                return a.normal_x * truth[0] + a.normal_y * truth[1] <
                       b.normal_x * truth[0] + b.normal_y * truth[1];
            });
        check(wall != estimate.walls.end() &&
                  wall->normal_x * truth[0] + wall->normal_y * truth[1] > std::cos(2 * pi / 180),
              "inferred normals should agree with the synthetic in-plane rotation");
        check(std::abs(wall->offset_m - fixture.distances[side]) < tolerance,
              "near supported walls should survive stronger later rectangular ghosts");
        check(wall->support_angles >= 18 && wall->longest_run_angles >= 8 &&
                  std::isfinite(wall->rms_residual_m),
              "wall evidence must expose support and residual instead of a fabricated probability");
    }
}

void rectangle_and_opaque_channels() {
    RectangleFixture fixture;
    const auto scan = synthetic(fixture);
    const auto estimate = viewer::infer_pool(scan);
    check_fixture(estimate, fixture, 0.15);
    unsigned retained_direct = 0, retained_ghost = 0;
    for (unsigned angle = 0; angle < 400; ++angle) {
        const auto arrival = fixture.arrival(angle);
        for (unsigned bin = 0; bin < bins; ++bin) {
            if (!estimate.retains(angle, bin)) {
                continue;
            }
            const double range = scan[angle][bin].center_range_m;
            if (std::abs(range - arrival.first) < 0.15) {
                ++retained_direct;
            }
            if (std::abs(range - 2 * arrival.first) < 0.15) {
                ++retained_ghost;
            }
        }
    }
    check(retained_direct > 300 && retained_ghost == 0,
          "the retained measured-bin mask should preserve wall evidence and reject the outer ghost "
          "band");
    auto short_inner_line = synthetic(fixture, 0.03, true, false);
    const auto inner_normal = fixture.normal(0);
    for (unsigned angle = 7; angle < 31; ++angle) {
        const double bearing = angle * 0.9 * pi / 180;
        const double cosine =
            inner_normal[0] * std::cos(bearing) + inner_normal[1] * std::sin(bearing);
        pulse(short_inner_line[angle], 2 / cosine, 0.03, 100, 2);
    }
    check_fixture(viewer::infer_pool(short_inner_line), fixture, 0.15);
    auto relabeled = scan;
    for (auto &profile : relabeled) {
        for (auto &bin : profile) {
            std::swap(bin.direct_intensity, bin.multipath_intensity);
        }
    }
    const auto same_signal = viewer::infer_pool(relabeled);
    check(same_signal.retained_bins == estimate.retained_bins &&
              same_signal.walls.size() == estimate.walls.size(),
          "relabeling channels without changing sums must not change inference");
    for (std::size_t wall = 0; wall < estimate.walls.size(); ++wall) {
        check(same_signal.walls[wall].offset_m == estimate.walls[wall].offset_m,
              "channel labels cannot act as simulator ground truth");
    }
    fixture.rotation_deg = 63.5;
    fixture.distances = {2.5, 6, 3.5, 4.5};
    check_fixture(viewer::infer_pool(synthetic(fixture)), fixture, 0.15);
}

void abstention_and_long_pulse() {
    const RectangleFixture fixture;
    const auto partial = viewer::infer_pool(synthetic(fixture, 0.03, false, false, 0));
    check(partial.status == viewer::PoolInferenceStatus::Partial && partial.walls.size() < 4,
          "a missing wall must not be filled in as a perfect rectangle");
    check(viewer::infer_pool(empty_scan()).status == viewer::PoolInferenceStatus::Insufficient,
          "empty signal must abstain");
    viewer::ScanProfiles no_metadata;
    check(viewer::infer_pool(no_metadata).walls.empty(),
          "fully empty profiles must abstain safely");
    auto noise = empty_scan();
    std::mt19937 random(99);
    for (auto &profile : noise) {
        for (unsigned point = 0; point < 4; ++point) {
            pulse(profile, 0.9 + (random() % 20000) * 0.001, 0.03, 1, 2);
        }
    }
    check(viewer::infer_pool(noise).status == viewer::PoolInferenceStatus::Insufficient,
          "isolated random returns without consecutive line support must abstain");
    auto sparse = synthetic(fixture, 0.03, false, false);
    for (unsigned angle = 10; angle < 400; ++angle) {
        sparse[angle].clear();
    }
    check(viewer::infer_pool(sparse).status == viewer::PoolInferenceStatus::Insufficient,
          "a sparse scan cannot establish supported walls");
    viewer::InferenceOptions long_pulse;
    long_pulse.pulse_width_m = 0.74;
    const auto broad = viewer::infer_pool(synthetic(fixture, 0.74, true, false), long_pulse);
    check_fixture(broad, fixture, 0.15);
    check(
        broad.range_uncertainty_m >= 0.74,
        "long pulses must expose conservative range uncertainty despite a small line-fit residual");
    auto malformed = empty_scan();
    malformed[0][0].direct_intensity = -1;
    bool rejected = false;
    try {
        viewer::infer_pool(malformed);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    check(rejected, "negative input energy must be rejected");
}

void configurable_strength_and_disjoint_runs() {
    RectangleFixture fixture;
    auto scan = synthetic(fixture, 0.03, true, false);
    for (unsigned angle = 0; angle < 400; ++angle) {
        pulse(scan[angle], fixture.arrival(angle).first + 0.15, 0.03, 0.1, 2);
    }
    std::array<std::size_t, 3> retained{};
    unsigned index = 0;
    for (const auto strength : {viewer::FilterStrength::Relaxed, viewer::FilterStrength::Balanced,
                                viewer::FilterStrength::Strict}) {
        viewer::InferenceOptions options;
        options.pulse_width_m = 0.03;
        options.minimum_range_m = 0.6;
        options.set_strength(strength);
        check(options.pulse_width_m == 0.03 && options.minimum_range_m == 0.6,
              "strength presets must preserve acquisition widths and minimum range");
        const auto estimate = viewer::infer_pool(scan, options);
        check_fixture(estimate, fixture, 0.15);
        for (const auto &mask : estimate.retained_bins) {
            retained[index] += static_cast<std::size_t>(std::count(mask.begin(), mask.end(), 1));
        }
        ++index;
    }
    check(retained[0] > retained[2] && retained[0] >= retained[1] && retained[1] >= retained[2],
          "Relaxed should retain more nearby measured signal than Strict in a fixed-wall fixture");
    fixture.rotation_deg = 0;
    auto interrupted = synthetic(fixture, 0.03, false, false);
    for (unsigned angle = 8; angle <= 20; ++angle) {
        for (auto &bin : interrupted[angle]) {
            bin.direct_intensity = 0;
        }
    }
    for (auto &bin : interrupted[30]) {
        bin.direct_intensity = 0;
    }
    const auto estimate = viewer::infer_pool(interrupted);
    const auto wall = std::find_if(estimate.walls.begin(), estimate.walls.end(),
                                   [](const auto &candidate) { return candidate.normal_x > 0.99; });
    check(wall != estimate.walls.end() && wall->segments.size() >= 2,
          "independently supported disjoint runs should be retained as separate segments");
    check(std::isnan(estimate.predicted_range_m[14]),
          "a large unsupported heading gap must not be extrapolated into a wall mask");
    check(std::isfinite(estimate.predicted_range_m[30]),
          "a small supported gap can be bridged by the Balanced preset");
    bool rejected = false;
    try {
        viewer::InferenceOptions options;
        options.set_strength(static_cast<viewer::FilterStrength>(99));
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    check(rejected, "invalid strength values must be rejected");
}

} // namespace

int main() {
    try {
        rectangle_and_opaque_channels();
        abstention_and_long_pulse();
        configurable_strength_and_disjoint_runs();
        std::cout << "PASS: signal-only rectangular prior, stronger ghosts, scatter, channel-sum "
                     "invariance, partial/empty abstention and long pulses\n";
        return 0;
    } catch (const std::exception &failure) {
        std::cerr << "FAIL: " << failure.what() << '\n';
        return 1;
    }
}
