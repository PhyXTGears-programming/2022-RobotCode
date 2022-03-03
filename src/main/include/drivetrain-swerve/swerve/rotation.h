// Copyright 2020 Robby Sammelson
// Used with permission

#include <cmath>
#include <utility>

namespace swervedrive {
namespace module_rotation {

constexpr double PI = 3.141592653589793238;
constexpr double PI_INV = 0.318309886183790672;

// units are radians
std::pair<double, bool> calculate_rotation_target (double wheel_target, double wheel_angle) {
    int half_rots_error = std::floor((wheel_angle - wheel_target)*PI_INV + 0.5);
    bool invert = half_rots_error % 2 != 0;
    double target = wheel_target + PI*half_rots_error;
    return {target, invert};
}

}
}
