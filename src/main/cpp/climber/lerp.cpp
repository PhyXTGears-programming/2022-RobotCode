#include "climber/lerp.h"

#include <cmath>

double lerp(double min, double max, double value) {
    return std::fmin(min + (max - min) * value, max); // makes sure the returned value never exceeds max
}