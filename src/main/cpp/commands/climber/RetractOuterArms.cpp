#include "commands/climber/RetractOuterArms.h"

const double kAcceptableExtensionError = 0.05;

RetractOuterArmsCommand::RetractOuterArmsCommand(OuterReach * outerArms, double targetExtension) {
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetExtension = targetExtension;
}

void RetractOuterArmsCommand::Initialize() {}

void RetractOuterArmsCommand::Execute() {
    if (mOuterArms->isMotor1NearTarget(mTargetExtension)) {
        mOuterArms->stop1();
    } else {
        mOuterArms->retract1();
    }
    if (mOuterArms->isMotor2NearTarget(mTargetExtension)) {
        mOuterArms->stop2();
    } else {
        mOuterArms->retract2();
    }
}

void RetractOuterArmsCommand::End(bool isInterrupted) {
    mOuterArms->stop1();
    mOuterArms->stop2();
}

bool RetractOuterArmsCommand::IsFinished() {
    return mOuterArms->isMotor1NearTarget(mTargetExtension) && mOuterArms->isMotor2NearTarget(mTargetExtension);
}