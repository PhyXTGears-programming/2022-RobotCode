#include "commands/climber/RotateOuterArms.h"

RotateOuterArmsCommand::RotateOuterArmsCommand(Climber * climber, double targetRotation) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetRotation = targetRotation;
}

void RotateOuterArmsCommand::Initialize() {
    mClimber->rotateOuter(mTargetRotation);
}

void RotateOuterArmsCommand::Execute() {}

void RotateOuterArmsCommand::End(bool isInterrupted) {}

bool RotateOuterArmsCommand::IsFinished() {
    return true;
}