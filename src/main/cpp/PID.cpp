#include "PID.h"

#include <algorithm>

PID::PID (
    double p, double i, double d,
    double ff, double acceptableError,
    double minOutput, double maxOutput, double izone
) {
    mProportional = p;
    mIntegral = i;
    mDeriviation = d;
    mFeedForward = ff;
    mAcceptableError = acceptableError;

    // These variables have values assigned in the header declaration of this function.
    mIZone = izone;
    mMinOutput = minOutput;
    mMaxOutput = mMaxOutput;
}

double PID::calculate (double current) {
    double output = 0.0;
    double error = mTarget - current;
    if (std::abs(error) <= mIZone) {
        mAccumulator += error;
    }

    output += error * mProportional;
    output += mAccumulator * mIntegral;
    output += (error - mPreviousError) * mDeriviation;

    if (std::abs(output) >= mAcceptableError) {
        output += std::copysign(mFeedForward, output);
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

void PID::setTarget (double target) {
    mTarget = target;
}