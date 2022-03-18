#include "commands/climber/RetractInnerArms.h"

const double kAcceptableExtensionError = 0.05;

RetractInnerArmsCommand::RetractInnerArmsCommand(ClimberInnerReach * innerArms, double targetExtension) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
    mTargetExtension = targetExtension;
}

void RetractInnerArmsCommand::Initialize() {}

void RetractInnerArmsCommand::Execute() {
    bool is1NearTarget = mInnerArms->isMotor1NearTarget(mTargetExtension);
    bool is2NearTarget = mInnerArms->isMotor2NearTarget(mTargetExtension);
    
    if (is1NearTarget) {
        mInnerArms->stop1();
    } else {
        mInnerArms->retract1();
    }
    if (is2NearTarget) {
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
    return mInnerArms->getMotor1Position() < mTargetExtension
        && mInnerArms->getMotor2Position() < mTargetExtension;
}