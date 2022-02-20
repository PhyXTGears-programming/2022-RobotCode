#include "commands/climber/RetractInnerArms.h"

const double kAcceptableExtensionError = 0.05;

RetractInnerArmsCommand::RetractInnerArmsCommand(Climber * climber, double targetExtension) {
    AddRequirements(climber);
    mClimber = climber;
    mTargetExtension = targetExtension;
}

void RetractInnerArmsCommand::Initialize() {}

void RetractInnerArmsCommand::Execute() {
    if (mClimber->isInner1NearTarget(mTargetExtension)) {
        mClimber->stopInner1();
    } else {
        mClimber->retractInner1();
    }
    if (mClimber->isInner2NearTarget(mTargetExtension)) {
        mClimber->stopInner2();
    } else {
        mClimber->retractInner2();
    }
}

void RetractInnerArmsCommand::End(bool isInterrupted) {
    mClimber->stopInner1();
    mClimber->stopInner2();
}

bool RetractInnerArmsCommand::IsFinished() {
    return mClimber->isInner1NearTarget(mTargetExtension) && mClimber->isInner2NearTarget(mTargetExtension);
}