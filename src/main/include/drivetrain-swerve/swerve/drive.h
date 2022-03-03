// Copyright 2020 Robby Sammelson
// Used with permission

#pragma once

#include <functional>
#include <initializer_list>
#include <vector>

#include "swerve_module.h"
#include "types.h"
#include "vector2.h"

namespace swervedrive {

template<class D, class F, class A, class M = default_math_functions>
class drive {
    public:
        drive (std::initializer_list<swerve_module<D, F, A>*> modules) {
            drive_modules = new std::vector<swerve_module<D, F, A>*>(modules);
        }

        void set_motion (vector2<decltype(D()*F())> velocity, decltype(A()*F()) angular_velocity, A robot_angle = A()) {
            // Rotate velocity for field oriented control
            auto v_x = velocity.get_x();
            auto v_y = velocity.get_y();
            auto sin_a = M::sin(-robot_angle);
            auto cos_a = M::cos(robot_angle);
            vector2<decltype(D()*F())> translational_component = {
                v_x*cos_a - v_y*sin_a,
                v_x*sin_a + v_y*cos_a
            };

            // Remove angular units from angular velocity
            F angular_velocity_no_angle = M::arc_length(angular_velocity/F(1)) * F(1) / D(1);

            // Get the function representing the vector field
            motion_function<D, F, A> motion_function = [=](vector2<D> pos) -> std::pair<decltype(D()*F()), A> {
                // Get rotational component of motion
                vector2<D> tangent {-1 * pos.get_y(), pos.get_x()};
                vector2<decltype(D()*F())> angularComponent = tangent * angular_velocity_no_angle;

                // Get total motion
                vector2<decltype(D()*F())> motion = translational_component + angularComponent;

                // Convert motion to polar coordinates
                return {
                    M::sqrt(motion.get_x()*motion.get_x() + motion.get_y()*motion.get_y()),
                    M::atan2(motion.get_y(), motion.get_x())
                };
            };

            // Send the function to each module
            for (auto module : *drive_modules) {
                module->set_motion(motion_function);
            }
        }

    private:
        std::vector<swerve_module<D, F, A>*> *drive_modules;
};

}
