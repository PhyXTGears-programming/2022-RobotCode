#include "commands/climber/ExtendOuterArms.h"

const double kAcceptableExtensionError = 0.05;

ExtendOuterArmsCommand::ExtendOuterArmsCommand(Climber * climber, double targetExtension) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetExtension = targetExtension;
}

void ExtendOuterArmsCommand::Initialize() {}

void ExtendOuterArmsCommand::Execute() {
    if (mClimber->isOuter1NearTarget(mTargetExtension)) {
        mClimber->stopOuter1();
    } else {
        mClimber->extendOuter1();
    }
    if (mClimber->isOuter2NearTarget(mTargetExtension)) {
        mClimber->stopOuter2();
    } else {
        mClimber->extendOuter2();
    }
}

void ExtendOuterArmsCommand::End(bool isInterrupted) {
    mClimber->stopOuter1();
    mClimber->stopOuter2();
}

bool ExtendOuterArmsCommand::IsFinished() {
    return mClimber->isOuter1NearTarget(mTargetExtension) && mClimber->isOuter2NearTarget(mTargetExtension);
}