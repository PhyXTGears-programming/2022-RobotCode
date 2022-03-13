#include "commands/climber/RetractInnerArms.h"

const double kAcceptableExtensionError = 0.05;

RetractInnerArmsCommand::RetractInnerArmsCommand(InnerReach * innerArms, double targetExtension) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
    mTargetExtension = targetExtension;
}

void RetractInnerArmsCommand::Initialize() {}

void RetractInnerArmsCommand::Execute() {
    if (mInnerArms->isMotor1NearTarget(mTargetExtension)) {
        mInnerArms->stop1();
    } else {
        mInnerArms->retract1();
    }
    if (mInnerArms->isMotor2NearTarget(mTargetExtension)) {
        mInnerArms->stop2();
    } else {
        mInnerArms->retract2();
    }
}

void RetractInnerArmsCommand::End(bool isInterrupted) {
    mInnerArms->stop1();
    mInnerArms->stop2();
}

bool RetractInnerArmsCommand::IsFinished() {
    return mInnerArms->isMotor1NearTarget(mTargetExtension) && mInnerArms->isMotor2NearTarget(mTargetExtension);
}