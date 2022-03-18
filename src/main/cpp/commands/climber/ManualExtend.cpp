#include "commands/climber/ManualExtend.h"

ManualExtendCommand::ManualExtendCommand(ClimberInnerReach * innerArms) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
    
    mRetract = new RetractInnerArmsCommand {mInnerArms, kRestingExtension};
}

void ManualExtendCommand::Initialize() {}

void ManualExtendCommand::Execute() {
    if (mInnerArms->isMotor1NearTarget(kTargetExtension)) {
        mInnerArms->stop1();
    } else {
        mInnerArms->extend1();
    }
    if (mInnerArms->isMotor2NearTarget(kTargetExtension)) {
        mInnerArms->stop2();
    } else {
        mInnerArms->extend2();
    }
}

void ManualExtendCommand::End(bool isInterrupted) {
    mRetract->Schedule();
}

bool ManualExtendCommand::IsFinished() {
    return mInnerArms->isMotor1NearTarget(kTargetExtension) && mInnerArms->isMotor2NearTarget(kTargetExtension);
}