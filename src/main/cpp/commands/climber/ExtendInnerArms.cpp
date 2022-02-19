#include "commands/climber/ExtendInnerArms.h"

const double kAcceptableExtensionError = 0.05;

ExtendInnerArmsCommand::ExtendInnerArmsCommand(Climber * climber, double targetExtension) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetExtension = targetExtension;
}

void ExtendInnerArmsCommand::Initialize() {
    mClimber->extendInner();
}

void ExtendInnerArmsCommand::Execute() {}

void ExtendInnerArmsCommand::End(bool isInterrupted) {
    mClimber->stopInner();
}

bool ExtendInnerArmsCommand::IsFinished() {
    return abs(mTargetExtension - mClimber->getInnerArmExtension()) < kAcceptableExtensionError;
}