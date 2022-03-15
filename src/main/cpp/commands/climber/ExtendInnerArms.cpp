#include "commands/climber/ExtendInnerArms.h"

ExtendInnerArmsCommand::ExtendInnerArmsCommand(ClimberInnerReach * innerArms, double targetExtension) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
    mTargetExtension = targetExtension;
}

void ExtendInnerArmsCommand::Initialize() {}

void ExtendInnerArmsCommand::Execute() {
    bool is1NearTarget = mInnerArms->isMotor1NearTarget(mTargetExtension);
    bool is2NearTarget = mInnerArms->isMotor2NearTarget(mTargetExtension);

    if (is1NearTarget) {
        mInnerArms->stop1();
    } else {
        mInnerArms->extend1();
    }
    if (is2NearTarget) {
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