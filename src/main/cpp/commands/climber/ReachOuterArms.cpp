#include "commands/climber/ReachOuterArms.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

ReachOuterArmsCommand::ReachOuterArmsCommand(ClimberOuterReach * outerArms, double targetPosition) {
    SetName("Reach Outer Arms");
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetPosition = targetPosition;

    mPid1.setTarget(targetPosition);
    mPid2.setTarget(targetPosition);
}

ReachOuterArmsCommand::ReachOuterArmsCommand(ClimberOuterReach * outerArms, double targetPosition, PID const & pid1, PID const & pid2) {
    SetName("Reach Outer Arms");
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetPosition = targetPosition;

    mPid1 = pid1;
    mPid2 = pid2;

    mPid1.setTarget(targetPosition);
    mPid2.setTarget(targetPosition);
}

void ReachOuterArmsCommand::Initialize() {
    mPid1.reset();
    mPid2.reset();
}

void ReachOuterArmsCommand::Execute() {
    double pos1 = mOuterArms->getMotor1Position();
    double pos2 = mOuterArms->getMotor2Position();

    double speed1 = mPid1.calculate(pos1);
    double speed2 = mPid2.calculate(pos2);

    frc::SmartDashboard::PutNumber("Reach Out 1: Speed", speed1);
    frc::SmartDashboard::PutNumber("Reach Out 2: Speed", speed2);

    mOuterArms->run1(speed1);
    mOuterArms->run2(speed2);
}

void ReachOuterArmsCommand::End(bool isInterrupted) {
    mOuterArms->stop1();
    mOuterArms->stop2();

    frc::SmartDashboard::PutNumber("Reach Out 1: Speed", 0.0);
    frc::SmartDashboard::PutNumber("Reach Out 2: Speed", 0.0);
}

bool ReachOuterArmsCommand::IsFinished() {
    bool isNear1 = std::abs(mPid1.getError()) < constants::climb::kAcceptablePositionError
        && std::abs(mPid1.getVelocityError()) < constants::climb::kAcceptableVelocityError;
    bool isNear2 = std::abs(mPid2.getError()) < constants::climb::kAcceptablePositionError
        && std::abs(mPid2.getVelocityError()) < constants::climb::kAcceptableVelocityError;

    frc::SmartDashboard::PutBoolean("Reach Out 1: Near Target", isNear1);
    frc::SmartDashboard::PutBoolean("Reach Out 2: Near Target", isNear2);

    return isNear1 && isNear2;
}

void ReachOuterArmsCommand::SetPid(double p, double i, double d, double ff) {
    SetP(p);
    SetI(i);
    SetD(d);
    SetFF(ff);
}

void ReachOuterArmsCommand::SetP(double p) {
    mPid1.setP(p);
    mPid2.setP(p);
}

void ReachOuterArmsCommand::SetI(double i) {
    mPid1.setI(i);
    mPid2.setI(i);
}

void ReachOuterArmsCommand::SetD(double d) {
    mPid1.setD(d);
    mPid2.setD(d);
}

void ReachOuterArmsCommand::SetFF(double ff) {
    mPid1.setFeedForward(ff);
    mPid2.setFeedForward(ff);
}
