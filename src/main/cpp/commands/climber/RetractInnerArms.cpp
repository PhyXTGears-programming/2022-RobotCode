#include "commands/climber/RetractInnerArms.h"

const double kAcceptableExtensionError = 0.05;

RetractInnerArmsCommand::RetractInnerArmsCommand(Climber * climber, double targetExtension) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetExtension = targetExtension;
}

void RetractInnerArmsCommand::Initialize() {
    mClimber->retractInner();
}

void RetractInnerArmsCommand::Execute() {}

void RetractInnerArmsCommand::End(bool isInterrupted) {
    mClimber->stopInner();
}

bool RetractInnerArmsCommand::IsFinished() {
    return abs(mTargetExtension - mClimber->getInnerArmExtension()) < kAcceptableExtensionError;
}