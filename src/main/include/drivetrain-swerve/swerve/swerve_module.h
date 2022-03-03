// Copyright 2020 Robby Sammelson
// Used with permission

#pragma once

#include <cmath>
#include <functional>

#include "types.h"
#include "vector2.h"

namespace swervedrive {

template<class D, class F, class A>
class swerve_module {
    public:
        swerve_module (vector2<D> position) {
            pos = position;
        }

        swerve_module (vector2<D> position, std::function<void(decltype(D()*F()), A)> drive_function) {
            pos = position;
            drive_lambda = drive_function;
        }

        void set_motion (motion_function<D, F, A> mF) {
            auto motion = mF(pos);
            drive(motion.first, motion.second);
        }

        virtual void drive (decltype(D()*F()) speed, A angle) {
            drive_lambda(speed, angle);
        }

    private:
        vector2<D> pos;
        std::function<void(decltype(D()*F()), A)> drive_lambda = [](decltype(D()*F()) speed, A angle) {};
};

}
