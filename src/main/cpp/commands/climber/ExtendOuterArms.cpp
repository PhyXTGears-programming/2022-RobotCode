#include "commands/climber/ExtendOuterArms.h"

const double kAcceptableExtensionError = 0.05;

ExtendOuterArmsCommand::ExtendOuterArmsCommand(OuterReach * outerArms, double targetExtension) {
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetExtension = targetExtension;
}

void ExtendOuterArmsCommand::Initialize() {}

void ExtendOuterArmsCommand::Execute() {
    bool is1NearTarget = mOuterArms->isMotor1NearTarget(mTargetExtension);
    bool is2NearTarget = mOuterArms->isMotor2NearTarget(mTargetExtension);
    
    if (is1NearTarget) {
        mOuterArms->stop1();
    } else {
        mOuterArms->extend1();
    }
    if (is2NearTarget) {
        mOuterArms->stop2();
    } else {
        mOuterArms->extend2();
    }
}

void ExtendOuterArmsCommand::End(bool isInterrupted) {
    mOuterArms->stop1();
    mOuterArms->stop2();
}

bool ExtendOuterArmsCommand::IsFinished() {
    return mOuterArms->isMotor1NearTarget(mTargetExtension) && mOuterArms->isMotor2NearTarget(mTargetExtension);
}