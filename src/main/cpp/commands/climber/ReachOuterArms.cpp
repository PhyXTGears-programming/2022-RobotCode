#include "commands/climber/ReachOuterArms.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

void initPid(frc2::PIDController &pid, double targetPosition) {
    pid.SetSetpoint(targetPosition);

    pid.SetIntegratorRange(-2.0, 2.0);
    pid.SetTolerance(0.1);
}

ReachOuterArmsCommand::ReachOuterArmsCommand(ClimberOuterReach * outerArms, double targetPosition) {
    SetName("Reach Outer Arms");
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetPosition = targetPosition;

    initPid(mPid1, targetPosition);
    initPid(mPid2, targetPosition);
    mCustomPID1.setTarget(targetPosition);
    mCustomPID2.setTarget(targetPosition);
}

void ReachOuterArmsCommand::Initialize() {
    mPid1.Reset();
    mPid2.Reset();
    mCustomPID1.reset();
    mCustomPID2.reset();
}

void ReachOuterArmsCommand::Execute() {
    double pos1 = mOuterArms->getMotor1Position();
    double pos2 = mOuterArms->getMotor2Position();

    double speed1 = std::clamp(mPid1.Calculate(pos1), -0.6, 0.6);
    double speed2 = std::clamp(mPid2.Calculate(pos2), -0.6, 0.6);
    double testSpeed1 = mCustomPID1.calculate(pos1);
    double testSpeed2 = mCustomPID2.calculate(pos2);

    // Add in feed forward if speed is above threshold.
    speed1 = (std::abs(speed1) >= 0.01)
        ? std::copysign(mFF, speed1) + speed1
        : 0.0;
    speed2 = (std::abs(speed2) >= 0.01)
        ? std::copysign(mFF, speed2) + speed2
        : 0.0;

    frc::SmartDashboard::PutNumber("Reach Out 1: Speed", speed1);
    frc::SmartDashboard::PutNumber("Reach Out 2: Speed", speed2);

    frc::SmartDashboard::PutNumber("Test Reach Out 1: Speed", testSpeed1);
    frc::SmartDashboard::PutNumber("Test Reach Out 2: Speed", testSpeed2);

    frc::SmartDashboard::PutNumber("Test Reach Error 1: Speed", speed1 - testSpeed1);
    frc::SmartDashboard::PutNumber("Test Reach Error 2: Speed", speed2 - testSpeed2);

    mOuterArms->run1(speed1);
    mOuterArms->run2(speed2);
}

void ReachOuterArmsCommand::End(bool isInterrupted) {
    mOuterArms->stop1();
    mOuterArms->stop2();

    frc::SmartDashboard::PutNumber("Test Reach Error 1: Speed", 0.0);
    frc::SmartDashboard::PutNumber("Test Reach Error 2: Speed", 0.0);
}

bool ReachOuterArmsCommand::IsFinished() {
    bool isNear1 = mOuterArms->isMotor1NearTarget(mTargetPosition) && std::abs(mPid1.GetVelocityError()) < 0.25;
    bool isNear2 = mOuterArms->isMotor2NearTarget(mTargetPosition) && std::abs(mPid2.GetVelocityError()) < 0.25;
    bool isTestNear1 = mOuterArms->isMotor2NearTarget(mTargetPosition);
    bool isTestNear2 = mOuterArms->isMotor2NearTarget(mTargetPosition);

    frc::SmartDashboard::PutBoolean("Reach Out 1: Near Target", isNear1);
    frc::SmartDashboard::PutBoolean("Reach Out 2: Near Target", isNear2);
    frc::SmartDashboard::PutBoolean("Reach Out Test 1: Near Target", isTestNear1);
    frc::SmartDashboard::PutBoolean("Reach Out Test 2: Near Target", isTestNear2);

    return isNear1 && isNear2;
}

void ReachOuterArmsCommand::SetPid(double p, double i, double d, double ff) {
    SetP(p);
    SetI(i);
    SetD(d);
    SetFF(ff);
}

void ReachOuterArmsCommand::SetP(double p) {
    mPid1.SetP(p);
    mPid2.SetP(p);
}

void ReachOuterArmsCommand::SetI(double i) {
    mPid1.SetI(i);
    mPid2.SetI(i);
}

void ReachOuterArmsCommand::SetD(double d) {
    mPid1.SetD(d);
    mPid2.SetD(d);
}

void ReachOuterArmsCommand::SetFF(double ff) {
    mFF = ff;
}
