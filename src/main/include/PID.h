#pragma once

#include <cmath>
#include <units/time.h>

class PID {
    public:
        PID (
            double p = 0.0, double i = 0.0, double d = 0.0,
            double ff = 0.0, double acceptableError = 0.005,
            double minOutput = -1.0, double maxOutput = 1.0, double izone = INFINITY,
            units::second_t period = 20_ms
        );

        PID (PID const &) = default;

        double calculate(double current);

        void setTarget(double target);
        void setP (double p);
        void setI (double i);
        void setD (double d);
        void setFeedForward (double ff);
        //void update(double p, double i, double d, double ff, double minOutput = -1.0, double maxOutput = 1.0, double izone = INFINITY);

        void reset();

        double getError();
        double getVelocityError();

    private:
        double mProportional, mIntegral, mDeriviation;
        double mFeedForward;

        double mTarget;

        double mPreviousError = 0.0;
        double mVelocityError = 0.0;

        double mAccumulator = 0.0;
        double mAcceptableError;
        double mIZone = INFINITY;

        double mMinOutput = -1.0;
        double mMaxOutput = 1.0;

        units::second_t mPeriod = 20_ms;
};