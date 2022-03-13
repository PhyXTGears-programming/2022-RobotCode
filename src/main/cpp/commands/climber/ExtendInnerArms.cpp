#include "commands/climber/ExtendInnerArms.h"

ExtendInnerArmsCommand::ExtendInnerArmsCommand(InnerReach * innerArms, double targetExtension) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
    mTargetExtension = targetExtension;
}

void ExtendInnerArmsCommand::Initialize() {}

void ExtendInnerArmsCommand::Execute() {
    if (mInnerArms->isMotor1NearTarget(mTargetExtension)) {
        mInnerArms->stop1();
    } else {
        mInnerArms->extend1();
    }
    if (mInnerArms->isMotor2NearTarget(mTargetExtension)) {
        mInnerArms->stop2();
    } else {
        mInnerArms->extend2();
    }
}

void ExtendInnerArmsCommand::End(bool isInterrupted) {
    mInnerArms->stop1();
    mInnerArms->stop2();
}

bool ExtendInnerArmsCommand::IsFinished() {
    return mInnerArms->isMotor1NearTarget(mTargetExtension) && mInnerArms->isMotor2NearTarget(mTargetExtension);
}