#include "commands/climber/RotateInnerArms.h"
#include "climber/lerp.h"

const double kAcceptableAngleError = 0.0001;
const double kMinSpeed = 0.2;
const double kMaxSpeed = 0.2;


RotateInnerArmsCommand::RotateInnerArmsCommand(InnerRotate * innerArms, double targetAngle) {
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
    mInnerArms->rotate(speed);
}

void RotateInnerArmsCommand::End(bool isInterrupted) {
    mInnerArms->rotate(0.0);
}

bool RotateInnerArmsCommand::IsFinished() {
    double armAngle = mInnerArms->getAngle();
    double err = mTargetAngle - armAngle;
    return abs(err) < kAcceptableAngleError;
}