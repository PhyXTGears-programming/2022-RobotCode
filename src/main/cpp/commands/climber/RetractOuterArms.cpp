#include "commands/climber/RetractOuterArms.h"

const double kAcceptableExtensionError = 0.05;

RetractOuterArmsCommand::RetractOuterArmsCommand(ClimberOuterReach * outerArms, double targetExtension) {
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetExtension = targetExtension;
}

void RetractOuterArmsCommand::Initialize() {}

void RetractOuterArmsCommand::Execute() {
    bool is1NearTarget = mOuterArms->isMotor1NearTarget(mTargetExtension);
    bool is2NearTarget = mOuterArms->isMotor2NearTarget(mTargetExtension);

    if (is1NearTarget) {
        mOuterArms->stop1();
    } else {
        mOuterArms->retract1();
    }
    if (is2NearTarget) {
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
    return mOuterArms->getMotor1Position() < mTargetExtension
        && mOuterArms->getMotor2Position() < mTargetExtension;
}