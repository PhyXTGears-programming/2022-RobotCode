#include "commands/climber/ReachOuterArms.h"

#include <cmath>
#include <iostream>

ReachOuterArmsCommand::ReachOuterArmsCommand(ClimberOuterReach * outerArms, double targetPosition) {
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetPosition = targetPosition;
}

void ReachOuterArmsCommand::Initialize() {}

void ReachOuterArmsCommand::Execute() {
    bool is1NearTarget = mOuterArms->isMotor1NearTarget(mTargetPosition);
    bool is2NearTarget = mOuterArms->isMotor2NearTarget(mTargetPosition);

    bool is1Extend = (mTargetPosition - mOuterArms->getMotor1Position()) > 0;
    bool is2Extend = (mTargetPosition - mOuterArms->getMotor2Position()) > 0;

    if (is1NearTarget) {
        mOuterArms->stop1();
    } else if (is1Extend) {
        mOuterArms->extend1();
    } else {
        mOuterArms->retract1();
    }

    if (is2NearTarget) {
        mOuterArms->stop2();
    } else if (is2Extend) {
        mOuterArms->extend2();
    } else {
        mOuterArms->retract2();
    }
}

void ReachOuterArmsCommand::End(bool isInterrupted) {
    mOuterArms->stop1();
    mOuterArms->stop2();
}

bool ReachOuterArmsCommand::IsFinished() {
    return mOuterArms->isMotor1NearTarget(mTargetPosition)
        && mOuterArms->isMotor2NearTarget(mTargetPosition);
}