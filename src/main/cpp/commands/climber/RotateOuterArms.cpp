#include "commands/climber/RotateOuterArms.h"
#include "climber/lerp.h"

const double kAcceptableAngleError = 0.1;
const double kMinSpeed = 0.05;
const double kMaxSpeed = 0.5;


RotateOuterArmsCommand::RotateOuterArmsCommand(Climber * climber, double targetAngle) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetAngle = targetAngle;
}

void RotateOuterArmsCommand::Initialize() {}

void RotateOuterArmsCommand::Execute() {
    double armAngle = mClimber->getOuterAngle();
    double err = mTargetAngle - armAngle;
    double speed = lerp(kMinSpeed, kMaxSpeed, abs(err/180)) * ((err > 0) ? -1.0 : 1.0); // abs(err/180) should be a value between 0 and 1
    mClimber->rotateOuter(speed);
}

void RotateOuterArmsCommand::End(bool isInterrupted) {}

bool RotateOuterArmsCommand::IsFinished() {
    double armAngle = mClimber->getOuterAngle();
    double err = mTargetAngle - armAngle;
    return abs(err) < kAcceptableAngleError;
}