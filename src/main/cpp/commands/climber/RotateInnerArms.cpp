#include "commands/climber/RotateInnerArms.h"

RotateInnerArmsCommand::RotateInnerArmsCommand(Climber * climber, double targetRotation) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetRotation = targetRotation;
}

void RotateInnerArmsCommand::Initialize() {
    mClimber->rotateInner(mTargetRotation);
}

void RotateInnerArmsCommand::Execute() {}

void RotateInnerArmsCommand::End(bool isInterrupted) {}

bool RotateInnerArmsCommand::IsFinished() {
    return true;
}