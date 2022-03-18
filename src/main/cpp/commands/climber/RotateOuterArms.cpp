#include "commands/climber/RotateOuterArms.h"
#include "climber/lerp.h"

#include <cmath>

#define SLOWZONE 0.5
#define IS_WITHIN_SLOWZONE(input) ((fabs(input) < SLOWZONE))

const double kAcceptableAngleError = 0.1;

RotateOuterArmsCommand::RotateOuterArmsCommand(
    ClimberOuterRotate * outerArms,
    double targetAngle,
    double minSpeed,
    double maxSpeed
) {
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetAngle = targetAngle;
    mMinSpeed = minSpeed;
    mMaxSpeed = maxSpeed;
}

void RotateOuterArmsCommand::Initialize() {
    mOuterArms->setMotorBrake();
}

void RotateOuterArmsCommand::Execute() {
    // Angles are not radians or degrees.  They are duty cycle values...
    // somewhere between -1.0 and 1.0 it seems.  So use error to set direction
    // of rotation, (+) is lean forward, (-) is lean backward.
    double armAngle = mOuterArms->getAngle();
    double err = mTargetAngle - armAngle;
    
    if (mTargetAngle > 0 && err > 0) {
        // If gravity won't pull arm toward angle (armAngle > 0) and movement toward
        // target is against gravity (target > 0 and err > 0), then drive motor.
        if (armAngle < 0.0) {
            mOuterArms->rotate(mMinSpeed);
        } else {
            mOuterArms->rotate(mMaxSpeed);
        }
    } else if (mTargetAngle < 0 && err < 0) {
        // If gravity won't pull arm toward angle (armAngle < 0) and movement toward
        // target is against gravity (target < 0 and err < 0), then drive motor.
        if (armAngle > 0.0) {
            mOuterArms->rotate(-mMinSpeed);
        } else {
            mOuterArms->rotate(-mMaxSpeed);
        }
    } else {
        mOuterArms->stop();
    }
}

void RotateOuterArmsCommand::End(bool isInterrupted) {
    mOuterArms->stop();
}

bool RotateOuterArmsCommand::IsFinished() {
    return false;
    // double armAngle = mOuterArms->getAngle();
    // double err = mTargetAngle - armAngle;
    // return std::abs(err) < kAcceptableAngleError;
}
