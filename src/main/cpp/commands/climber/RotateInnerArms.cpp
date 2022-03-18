#include "commands/climber/RotateInnerArms.h"
#include "climber/lerp.h"

#include <cmath>

#define SLOWZONE 0.5
#define IS_WITHIN_SLOWZONE(input) ((fabs(input) < SLOWZONE))

const double kAcceptableAngleError = 0.1;


RotateInnerArmsCommand::RotateInnerArmsCommand(
    ClimberInnerRotate * innerArms,
    double targetAngle,
    double minSpeed,
    double maxSpeed
) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
    mTargetAngle = targetAngle;
    mMinSpeed = minSpeed;
    mMaxSpeed = maxSpeed;
}

void RotateInnerArmsCommand::Initialize() {
    mInnerArms->setMotorBrake();
}

void RotateInnerArmsCommand::Execute() {
    // Angles are not radians or degrees.  They are duty cycle values...
    // somewhere between -1.0 and 1.0 it seems.  So use error to set direction
    // of rotation, (+) is lean forward, (-) is lean backward.
    double armAngle = mInnerArms->getAngle();
    double err = mTargetAngle - armAngle;

    if (mTargetAngle > 0 && err > 0) {
        // If gravity won't pull arm toward angle (armAngle > 0) and movement toward
        // target is against gravity (target > 0 and err > 0), then drive motor.
        if (armAngle < 0.0) {
            mInnerArms->rotate(mMinSpeed);
        } else {
            mInnerArms->rotate(mMaxSpeed);
        }
    } else if (mTargetAngle < 0 && err < 0) {
        // If gravity won't pull arm toward angle (armAngle < 0) and movement toward
        // target is against gravity (target < 0 and err < 0), then drive motor.
        if (armAngle > 0.0) {
            mInnerArms->rotate(-mMinSpeed);
        } else {
            mInnerArms->rotate(-mMaxSpeed);
        }
    } else {
        mInnerArms->stop();
    }
}

void RotateInnerArmsCommand::End(bool isInterrupted) {
    mInnerArms->stop();
}

bool RotateInnerArmsCommand::IsFinished() {
    return false;
    // double armAngle = mInnerArms->getAngle();
    // double err = mTargetAngle - armAngle;
    // return std::abs(err) < kAcceptableAngleError;
}