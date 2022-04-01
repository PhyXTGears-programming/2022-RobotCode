#include "PID.h"

#include <algorithm>
#include <iostream>

PID::PID (
    double p, double i, double d,
    double ff, double acceptableError,
    double minOutput, double maxOutput, double izone,
    units::second_t period
) {
    mProportional = p;
    mIntegral = i;
    mDeriviation = d;
    mFeedForward = ff;
    mAcceptableError = acceptableError;

    // These variables have values assigned in the header declaration of this function.
    mIZone = izone;
    mMinOutput = minOutput;
    mMaxOutput = maxOutput;

    if (0_s <= period) {
        std::cerr << "PID period must be above zero" << std::endl;
        mPeriod = 20_ms;
    } else {
        mPeriod = period;
    }
}

double PID::calculate (double current) {
    double output = 0.0;
    double error = mTarget - current;
    if (std::abs(error) <= mIZone) {
        mAccumulator += error * mPeriod.value();
    }

    output += error * mProportional;
    output += mAccumulator * mIntegral;
    output += (error - mPreviousError) / mPeriod.value() * mDeriviation;

    if (std::abs(output) >= mAcceptableError) {
        output += std::copysign(mFeedForward, output);
    } else {
        output = 0.0;
    }

    mPreviousError = error;
    return std::clamp(output, mMinOutput, mMaxOutput);
}

void PID::setTarget (double target) {
    mTarget = target;
}

void PID::setP (double p) {
    mProportional = p;
}

void PID::setI (double i) {
    mIntegral = i;
}

void PID::setD (double d) {
    mDeriviation = d;
}

void PID::setFeedForward (double ff) {
    mFeedForward = ff;
}

void PID::reset () {
    mPreviousError = 0.0;
    mAccumulator = 0.0;
}

double PID::getError () {
    return mPreviousError;
}