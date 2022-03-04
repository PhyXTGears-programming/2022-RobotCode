#include "commands/climber/ExtendInnerArms.h"

ExtendInnerArmsCommand::ExtendInnerArmsCommand(Climber * climber, double targetExtension) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetExtension = targetExtension;
}

void ExtendInnerArmsCommand::Initialize() {}

void ExtendInnerArmsCommand::Execute() {
    if (mClimber->isInner1NearTarget(mTargetExtension)) {
        mClimber->stopInner1();
    } else {
        mClimber->extendInner1();
    }
    if (mClimber->isInner2NearTarget(mTargetExtension)) {
        mClimber->stopInner2();
    } else {
        mClimber->extendInner2();
    }
}

void ExtendInnerArmsCommand::End(bool isInterrupted) {
    mClimber->stopInner1();
    mClimber->stopInner2();
}

bool ExtendInnerArmsCommand::IsFinished() {
    return mClimber->isInner1NearTarget(mTargetExtension) && mClimber->isInner2NearTarget(mTargetExtension);
}