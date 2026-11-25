#include "pool_coordinates.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void check(bool condition, const char *message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const char *message) {
    check(std::isfinite(actual) && std::abs(actual - expected) < 1e-9, message);
}

viewer::EstimatedWall wall(double x, double y, double distance) {
    viewer::EstimatedWall result;
    result.normal_x = x;
    result.normal_y = y;
    result.offset_m = distance;
    result.support_angles = 30;
    return result;
}

viewer::PoolEstimate rectangle(double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    viewer::PoolEstimate estimate;
    estimate.status = viewer::PoolInferenceStatus::SupportedRectangle;
    estimate.walls = {wall(c, s, 7), wall(-c, -s, 3), wall(-s, c, 2), wall(s, -c, 6)};
    return estimate;
}

} // namespace

int main() {
    try {
        constexpr double pi = 3.14159265358979323846;
        const auto full = viewer::pool_coordinates(rectangle(0));
        check(full.axes[0] && full.axes[1] && full.center_xy_m,
              "four walls observe two internal coordinates");
        near(full.axes[0]->sonar_from_center_m, -2, "sonar X is negative center projection");
        near(full.axes[1]->sonar_from_center_m, 2, "sonar Y is negative center projection");
        near((*full.center_xy_m)[0], 2, "center X from opposing distances");
        near((*full.center_xy_m)[1], -2, "center Y from opposing distances");

        for (const double degrees : {20.0, 44.999, 45.0, 45.001, 70.0}) {
            const double angle = degrees * pi / 180;
            auto estimate = rectangle(angle);
            const auto forward = viewer::pool_coordinates(estimate);
            std::reverse(estimate.walls.begin(), estimate.walls.end());
            const auto reversed = viewer::pool_coordinates(estimate);
            check(forward.axes[0] && forward.axes[1] && forward.center_xy_m,
                  "rotated rectangle must retain both independent coordinates");
            const auto &x = forward.axes[0]->axis;
            const auto &y = forward.axes[1]->axis;
            check(x[0] >= 0 && y[1] >= 0, "axes orient toward their positive local names");
            check(std::abs(x[0]) + std::abs(y[1]) + 1e-10 >= std::abs(y[0]) + std::abs(x[1]),
                  "joint assignment maximizes combined X/Y alignment, including near 45 degrees");
            near((*forward.center_xy_m)[0], 2 * std::cos(angle) + 2 * std::sin(angle),
                 "rotated center satisfies both inferred constraints");
            near((*forward.center_xy_m)[1], 2 * std::sin(angle) - 2 * std::cos(angle),
                 "rotated center is independent of coordinate naming");
            for (unsigned slot = 0; slot < 2; ++slot) {
                near(forward.axes[slot]->axis[0], reversed.axes[slot]->axis[0],
                     "wall ordering must not rename an axis");
                near(forward.axes[slot]->axis[1], reversed.axes[slot]->axis[1],
                     "wall ordering preserves orientation");
                near(forward.axes[slot]->sonar_from_center_m,
                     reversed.axes[slot]->sonar_from_center_m,
                     "wall ordering preserves signed displacement");
            }
        }

        auto partial = rectangle(0);
        partial.status = viewer::PoolInferenceStatus::Partial;
        partial.walls.pop_back();
        const auto one_axis = viewer::pool_coordinates(partial);
        check(one_axis.axes[0] && !one_axis.axes[1] && !one_axis.center_xy_m,
              "a missing opposing wall must not invent its coordinate or full center");
        partial.walls = {wall(1, 0, 7), wall(0, 1, 2)};
        const auto adjacent = viewer::pool_coordinates(partial);
        check(!adjacent.axes[0] && !adjacent.axes[1] && !adjacent.center_xy_m,
              "adjacent walls alone do not determine pool-center coordinates");
        partial.walls = {wall(1, 0, 7), wall(-1, 0, 3), wall(.8, .6, 2), wall(-.8, -.6, 6)};
        check(!viewer::pool_coordinates(partial).center_xy_m,
              "non-orthogonal plane pairs must not fabricate a rectangular center");
        check(!viewer::pool_coordinates(viewer::PoolEstimate{}).center_xy_m,
              "empty evidence leaves internal coordinates unknown");
        std::cout << "PASS: opposing-wall internal coordinates, rotated axis assignment and "
                     "missing-axis observability\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
