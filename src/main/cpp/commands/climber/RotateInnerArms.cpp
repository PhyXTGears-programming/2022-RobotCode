#include "commands/climber/RotateInnerArms.h"
#include "climber/lerp.h"

#include <cmath>

#define SLOWZONE 0.5
#define IS_WITHIN_SLOWZONE(input) ((fabs(input) < SLOWZONE))

const double kAcceptableAngleError = 0.1;
const double kMinSpeed = 0.2;
const double kMaxSpeed = 0.2;


RotateInnerArmsCommand::RotateInnerArmsCommand(ClimberInnerRotate * innerArms, double targetAngle) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
    mTargetAngle = targetAngle;
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
    double speed = std::copysign(kMinSpeed, err);

    if (mTargetAngle > 0 && err > 0) {
        if (IS_WITHIN_SLOWZONE(err)) {
            mInnerArms->rotate(kMinSpeed);
        } else {
            mInnerArms->rotate(kMaxSpeed);
        }
    } else if (mTargetAngle < 0 && err < 0) {
        if (IS_WITHIN_SLOWZONE(err)) {
            mInnerArms->rotate(-kMinSpeed);
        } else {
            mInnerArms->rotate(-kMaxSpeed);
        }
    } else {
        mInnerArms->stop();
    }
}

void RotateInnerArmsCommand::End(bool isInterrupted) {
    mInnerArms->stop();
}

bool RotateInnerArmsCommand::IsFinished() {
    double armAngle = mInnerArms->getAngle();
    double err = mTargetAngle - armAngle;
    return std::abs(err) < kAcceptableAngleError;
}
