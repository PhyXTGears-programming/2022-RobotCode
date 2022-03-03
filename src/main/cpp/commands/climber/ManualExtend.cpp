#include "commands/climber/ManualExtend.h"

ManualExtendCommand::ManualExtendCommand(Climber * climber) {
    mClimber = climber;
    
    mRetract = new RetractInnerArmsCommand {mClimber, kRestingExtension};
}

void ManualExtendCommand::Initialize() {}

void ManualExtendCommand::Execute() {
    if (mClimber->isInner1NearTarget(kTargetExtension)) {
        mClimber->stopInner1();
    } else {
        mClimber->extendInner1();
    }
    if (mClimber->isInner2NearTarget(kTargetExtension)) {
        mClimber->stopInner2();
    } else {
        mClimber->extendInner2();
    }
}

void ManualExtendCommand::End(bool isInterrupted) {
    mRetract->Schedule();
}

bool ManualExtendCommand::IsFinished() {
    return mClimber->isInner1NearTarget(kTargetExtension) && mClimber->isInner2NearTarget(kTargetExtension);
}