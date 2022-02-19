#include "commands/climber/ExtendOuterArms.h"

const double kAcceptableExtensionError = 0.05;

ExtendOuterArmsCommand::ExtendOuterArmsCommand(Climber * climber, double targetExtension) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetExtension = targetExtension;
}

void ExtendOuterArmsCommand::Initialize() {
    mClimber->extendOuter();
}

void ExtendOuterArmsCommand::Execute() {}

void ExtendOuterArmsCommand::End(bool isInterrupted) {
    mClimber->stopOuter();
}

bool ExtendOuterArmsCommand::IsFinished() {
    return abs(mTargetExtension - mClimber->getOuterArmExtension()) < kAcceptableExtensionError;
}