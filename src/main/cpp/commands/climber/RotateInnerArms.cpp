#include "commands/climber/RotateInnerArms.h"
#include "climber/lerp.h"

const double kAcceptableAngleError = 0.1;
const double kMinSpeed = 0.05;
const double kMaxSpeed = 0.5;


RotateInnerArmsCommand::RotateInnerArmsCommand(Climber * climber, double targetAngle) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetAngle = targetAngle;
}

void RotateInnerArmsCommand::Initialize() {}

void RotateInnerArmsCommand::Execute() {
    double armAngle = mClimber->getInnerAngle();
    double err = mTargetAngle - armAngle;
    double speed = lerp(kMinSpeed, kMaxSpeed, abs(err/180)) * ((err > 0) ? -1.0 : 1.0);
    mClimber->rotateInner(speed);
}

void RotateInnerArmsCommand::End(bool isInterrupted) {}

bool RotateInnerArmsCommand::IsFinished() {
    double armAngle = mClimber->getInnerAngle();
    double err = mTargetAngle - armAngle;
    return abs(err) < kAcceptableAngleError;
}