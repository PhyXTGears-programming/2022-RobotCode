#include "commands/climber/ReachInnerArms.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

ReachInnerArmsCommand::ReachInnerArmsCommand(ClimberInnerReach * outerArms, double targetPosition) {
    SetName("Reach Inner Arms");
    AddRequirements(outerArms);
    mInnerArms = outerArms;
    mTargetPosition = targetPosition;

    mPid1.setTarget(targetPosition);
    mPid2.setTarget(targetPosition);
}

ReachInnerArmsCommand::ReachInnerArmsCommand(ClimberInnerReach * outerArms, double targetPosition, PID const & pid1, PID const & pid2) {
    SetName("Reach Inner Arms");
    AddRequirements(outerArms);
    mInnerArms = outerArms;
    mTargetPosition = targetPosition;

    mPid1 = pid1;
    mPid2 = pid2;

    mPid1.setTarget(targetPosition);
    mPid2.setTarget(targetPosition);
}

void ReachInnerArmsCommand::Initialize() {
    mPid1.reset();
    mPid2.reset();
}

void ReachInnerArmsCommand::Execute() {
    double pos1 = mInnerArms->getMotor1Position();
    double pos2 = mInnerArms->getMotor2Position();

    double speed1 = mPid1.calculate(pos1);
    double speed2 = mPid2.calculate(pos2);

    frc::SmartDashboard::PutNumber("Reach In 1: Speed", speed1);
    frc::SmartDashboard::PutNumber("Reach In 2: Speed", speed2);

    mInnerArms->run1(speed1);
    mInnerArms->run2(speed2);
}

void ReachInnerArmsCommand::End(bool isInterrupted) {
    mInnerArms->stop1();
    mInnerArms->stop2();

    frc::SmartDashboard::PutNumber("Reach In 1: Speed", 0.0);
    frc::SmartDashboard::PutNumber("Reach In 2: Speed", 0.0);
}

bool ReachInnerArmsCommand::IsFinished() {
    bool isNear1 = std::abs(mPid1.getError()) < constants::climb::kAcceptablePositionError
        && std::abs(mPid1.getVelocityError()) < constants::climb::kAcceptableVelocityError;
    bool isNear2 = std::abs(mPid2.getError()) < constants::climb::kAcceptablePositionError
        && std::abs(mPid2.getVelocityError()) < constants::climb::kAcceptableVelocityError;

    frc::SmartDashboard::PutBoolean("Reach In 1: Near Target", isNear1);
    frc::SmartDashboard::PutBoolean("Reach In 2: Near Target", isNear2);

    return isNear1 && isNear2;
}

void ReachInnerArmsCommand::SetPid(double p, double i, double d, double ff) {
    SetP(p);
    SetI(i);
    SetD(d);
    SetFF(ff);
}

void ReachInnerArmsCommand::SetP(double p) {
    mPid1.setP(p);
    mPid2.setP(p);
}

void ReachInnerArmsCommand::SetI(double i) {
    mPid1.setI(i);
    mPid2.setI(i);
}

void ReachInnerArmsCommand::SetD(double d) {
    mPid1.setD(d);
    mPid2.setD(d);
}

void ReachInnerArmsCommand::SetFF(double ff) {
    mPid1.setFeedForward(ff);
    mPid2.setFeedForward(ff);
}
