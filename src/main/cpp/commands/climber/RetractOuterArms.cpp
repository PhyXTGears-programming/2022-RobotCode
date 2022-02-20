#include "commands/climber/RetractOuterArms.h"

const double kAcceptableExtensionError = 0.05;

RetractOuterArmsCommand::RetractOuterArmsCommand(Climber * climber, double targetExtension) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetExtension = targetExtension;
}

void RetractOuterArmsCommand::Initialize() {}

void RetractOuterArmsCommand::Execute() {
    if (mClimber->isOuter1NearTarget(mTargetExtension)) {
        mClimber->stopOuter1();
    } else {
        mClimber->retractOuter1();
    }
    if (mClimber->isOuter2NearTarget(mTargetExtension)) {
        mClimber->stopOuter2();
    } else {
        mClimber->retractOuter2();
    }
}

void RetractOuterArmsCommand::End(bool isInterrupted) {
    mClimber->stopOuter1();
    mClimber->stopOuter2();
}

bool RetractOuterArmsCommand::IsFinished() {
    return mClimber->isOuter1NearTarget(mTargetExtension) && mClimber->isOuter2NearTarget(mTargetExtension);
}