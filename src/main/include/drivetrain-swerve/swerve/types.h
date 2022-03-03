// Copyright 2020 Robby Sammelson
// Used with permission

#pragma once

#include <functional>
#include <utility>
#include <cmath>

#include "vector2.h"

namespace swervedrive {

template<class D, class F, class A>
using motion_function = std::function<std::pair<decltype(D()*F()), A>(vector2<D>)>;

struct default_math_functions {
    static double sqrt (double a) { return std::sqrt(a); }; // (distance^2*frequency^2 -> distance*frequency)
    static double atan2 (double y, double x) { return std::atan2(y, x); }; // (distance*frequency, distance*frequency -> angle)
    static double sin (double a) { return std::sin(a); }; // (angle -> unitless)
    static double cos (double a) { return std::cos(a); }; // (angle -> unitless)
    static double arc_length (double a) { return a; }; // return the arc length on a unit circle (angle -> unitless)
};

}
