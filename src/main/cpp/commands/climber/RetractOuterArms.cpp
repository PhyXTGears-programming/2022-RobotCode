#include "commands/climber/RetractOuterArms.h"

const double kAcceptableExtensionError = 0.05;

RetractOuterArmsCommand::RetractOuterArmsCommand(Climber * climber, double targetExtension) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetExtension = targetExtension;
}

void RetractOuterArmsCommand::Initialize() {
    mClimber->retractOuter();
}

void RetractOuterArmsCommand::Execute() {}

void RetractOuterArmsCommand::End(bool isInterrupted) {
    mClimber->stopOuter();
}

bool RetractOuterArmsCommand::IsFinished() {
    return abs(mTargetExtension - mClimber->getOuterArmExtension()) < kAcceptableExtensionError;
}