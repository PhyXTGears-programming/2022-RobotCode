#pragma once

#include <cmath>

class PID {
    public:
        PID (
            double p, double i, double d,
            double ff, double acceptableError,
            double minOutput = -1.0, double maxOutput = 1.0, double izone = INFINITY
        );
        double calculate(double current);

        void setTarget(double target);

        void reset();

    private:
        double mProportional, mIntegral, mDeriviation;
        double mFeedForward;
        double mTarget;
        double mPreviousError = 0.0;
        double mAccumulator = 0.0;
        double mAcceptableError;
        double mIZone = INFINITY;
        double mMinOutput = -1.0;
        double mMaxOutput = 1.0;
};