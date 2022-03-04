#include "commands/climber/RotateOuterArms.h"
#include "climber/lerp.h"

const double kAcceptableAngleError = 0.0001;
const double kMinSpeed = 0.2;
const double kMaxSpeed = 0.2;


RotateOuterArmsCommand::RotateOuterArmsCommand(Climber * climber, double targetAngle) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetAngle = targetAngle;
}

void RotateOuterArmsCommand::Initialize() {
    mClimber->setRotateMotorsBrake();
}

void RotateOuterArmsCommand::Execute() {
    // Angles are not radians or degrees.  They are duty cycle values...
    // somewhere between -1.0 and 1.0 it seems.  So use error to set direction
    // of rotation, (+) is lean forward, (-) is lean backward.
    double armAngle = mClimber->getOuterAngle();
    double err = mTargetAngle - armAngle;
    double speed = std::copysign(kMinSpeed, err);
    mClimber->rotateOuter(speed);
}

void RotateOuterArmsCommand::End(bool isInterrupted) {
    mClimber->rotateOuter(0.0);
}

bool RotateOuterArmsCommand::IsFinished() {
    double armAngle = mClimber->getOuterAngle();
    double err = mTargetAngle - armAngle;
    return abs(err) < kAcceptableAngleError;
}