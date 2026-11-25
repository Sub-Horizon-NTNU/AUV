#pragma once

#include "inference.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace viewer {

struct AxisPosition {
    std::array<double, 2> axis{};
    double sonar_from_center_m{};
};

struct PoolCoordinates {
    // Slots are assigned jointly: closest to positive sonar-local X and Y respectively.
    // A missing opposing wall leaves that coordinate unobserved.
    std::array<std::optional<AxisPosition>, 2> axes{};
    std::optional<std::array<double, 2>> center_xy_m;
};

/**
 * Derives internal, center-relative coordinates from supported opposing wall pairs.
 * Inputs contain no vehicle pose, pool dimensions or world map. Axes describe the
 * inferred pool in the sonar-local XY frame; the sonar is the local origin.
 * Five-degree opposition/orthogonality tolerances accommodate fitted normals.
 * This is a geometric hypothesis, not a world position or localization accuracy.
 */
inline PoolCoordinates pool_coordinates(const PoolEstimate &estimate) {
    struct Pair {
        std::array<double, 2> axis;
        double center_projection;
        unsigned support;
    };
    constexpr double cosine_tolerance = 0.9961946980917455; // cos(5 degrees)
    constexpr double sine_tolerance = 0.0871557427476582;   // sin(5 degrees)
    std::vector<Pair> pairs;
    for (std::size_t first = 0; first < estimate.walls.size(); ++first) {
        const auto &a = estimate.walls[first];
        const double norm_a = std::hypot(a.normal_x, a.normal_y);
        if (!std::isfinite(norm_a) || norm_a <= 0 || !std::isfinite(a.offset_m) ||
            a.offset_m <= 0) {
            continue;
        }
        for (std::size_t second = first + 1; second < estimate.walls.size(); ++second) {
            const auto &b = estimate.walls[second];
            const double norm_b = std::hypot(b.normal_x, b.normal_y);
            if (!std::isfinite(norm_b) || norm_b <= 0 || !std::isfinite(b.offset_m) ||
                b.offset_m <= 0) {
                continue;
            }
            const std::array<double, 2> na{a.normal_x / norm_a, a.normal_y / norm_a};
            const std::array<double, 2> nb{b.normal_x / norm_b, b.normal_y / norm_b};
            if (na[0] * nb[0] + na[1] * nb[1] > -cosine_tolerance) {
                continue;
            }
            std::array<double, 2> axis{na[0] - nb[0], na[1] - nb[1]};
            const double norm = std::hypot(axis[0], axis[1]);
            axis[0] /= norm;
            axis[1] /= norm;
            double projection = (a.offset_m / norm_a - b.offset_m / norm_b) / 2;
            // Canonical sign makes assignment independent of which opposing wall came first.
            if (axis[0] < -1e-12 || (std::abs(axis[0]) <= 1e-12 && axis[1] < 0)) {
                axis[0] = -axis[0];
                axis[1] = -axis[1];
                projection = -projection;
            }
            pairs.push_back({axis, projection, std::min(a.support_angles, b.support_angles)});
        }
    }

    PoolCoordinates result;
    if (pairs.empty()) {
        return result;
    }
    const auto assign = [&](const Pair &pair, unsigned slot) {
        auto axis = pair.axis;
        double projection = pair.center_projection;
        if (axis[slot] < 0) {
            axis[0] = -axis[0];
            axis[1] = -axis[1];
            projection = -projection;
        }
        result.axes[slot] = AxisPosition{axis, -projection};
    };

    const Pair *first_axis = nullptr;
    const Pair *second_axis = nullptr;
    double best_alignment = -1;
    unsigned best_support = 0;
    for (std::size_t first = 0; first < pairs.size(); ++first) {
        for (std::size_t second = first + 1; second < pairs.size(); ++second) {
            const auto &a = pairs[first];
            const auto &b = pairs[second];
            if (std::abs(a.axis[0] * b.axis[0] + a.axis[1] * b.axis[1]) > sine_tolerance) {
                continue;
            }
            const double ab = std::abs(a.axis[0]) + std::abs(b.axis[1]);
            const double ba = std::abs(b.axis[0]) + std::abs(a.axis[1]);
            const double alignment = std::max(ab, ba);
            const unsigned support = a.support + b.support;
            if (alignment > best_alignment + 1e-12 ||
                (std::abs(alignment - best_alignment) <= 1e-12 && support > best_support)) {
                // At the 45-degree tie, prefer the X axis pointing toward local +Y.
                const bool a_is_x = std::abs(ab - ba) <= 1e-12 ? a.axis[1] >= b.axis[1] : ab > ba;
                first_axis = a_is_x ? &a : &b;
                second_axis = a_is_x ? &b : &a;
                best_alignment = alignment;
                best_support = support;
            }
        }
    }
    if (!first_axis) {
        const Pair *best = &pairs.front();
        for (const auto &pair : pairs) {
            if (pair.support > best->support) {
                best = &pair;
            }
        }
        assign(*best, std::abs(best->axis[0]) >= std::abs(best->axis[1]) ? 0U : 1U);
        return result;
    }
    assign(*first_axis, 0);
    assign(*second_axis, 1);
    const auto &a = *result.axes[0];
    const auto &b = *result.axes[1];
    const double determinant = a.axis[0] * b.axis[1] - a.axis[1] * b.axis[0];
    const double ca = -a.sonar_from_center_m;
    const double cb = -b.sonar_from_center_m;
    result.center_xy_m = std::array<double, 2>{(ca * b.axis[1] - a.axis[1] * cb) / determinant,
                                               (a.axis[0] * cb - ca * b.axis[0]) / determinant};
    return result;
}

} // namespace viewer
