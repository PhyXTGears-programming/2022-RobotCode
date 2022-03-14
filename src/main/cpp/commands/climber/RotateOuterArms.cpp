#include "commands/climber/RotateOuterArms.h"
#include "climber/lerp.h"

const double kAcceptableAngleError = 0.0001;
const double kMinSpeed = 0.2;
const double kMaxSpeed = 0.2;


RotateOuterArmsCommand::RotateOuterArmsCommand(OuterRotate * outerArms, double targetAngle) {
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetAngle = targetAngle;
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
    double speed = std::copysign(kMinSpeed, err);
    mOuterArms->rotate(speed);
}

void RotateOuterArmsCommand::End(bool isInterrupted) {
    mOuterArms->stop();
}

bool RotateOuterArmsCommand::IsFinished() {
    double armAngle = mOuterArms->getAngle();
    double err = mTargetAngle - armAngle;
    return abs(err) < kAcceptableAngleError;
}