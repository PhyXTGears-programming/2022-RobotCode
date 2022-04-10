#include "climber/OuterReach.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>

ClimberOuterReach::ClimberOuterReach(std::shared_ptr<cpptoml::table> toml) {
    config.extendSpeed = toml->get_qualified_as<double>("extendSpeed").value_or(0.5);
    config.retractSpeed = toml->get_qualified_as<double>("retractSpeed").value_or(-0.5);
    config.inchesPerRevolution = toml->get_qualified_as<double>("inchesPerRevolution").value_or(0.128325);
    config.friction.outerStaticFriction = toml->get_qualified_as<double>("outerStaticFriction").value_or(0.0);
    config.friction.outerStaticFrictionWithLoad = toml->get_qualified_as<double>("outerStaticFrictionWithLoad").value_or(0.0);

    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mMotor1.SetInverted(true);
    mMotor2.SetInverted(false);

    mMotor1.SetSmartCurrentLimit(40);
    mMotor2.SetSmartCurrentLimit(40);

    mEncoder1.SetPosition(0.0);
    mEncoder2.SetPosition(0.0);
}

void ClimberOuterReach::Periodic() {
    frc::SmartDashboard::PutNumber("Clmb Out R Pos", getMotor1Position());
    frc::SmartDashboard::PutNumber("Clmb Out L Pos", getMotor2Position());

    frc::SmartDashboard::PutNumber("Clmb Out R Vel", mEncoder1.GetVelocity());
    frc::SmartDashboard::PutNumber("Clmb Out L Vel", mEncoder2.GetVelocity());
}

void ClimberOuterReach::extend1() {
    double bonus = std::copysign(config.friction.outerStaticFriction, config.extendSpeed);
    mMotor1.Set(config.extendSpeed + bonus);
}

void ClimberOuterReach::extend2() {
    double bonus = std::copysign(config.friction.outerStaticFriction, config.extendSpeed);
    mMotor2.Set(config.extendSpeed + bonus);
}

void ClimberOuterReach::retract1() {
    double bonus = (mIsUnderLoad)
        ? config.friction.outerStaticFrictionWithLoad
        : config.friction.outerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor1.Set(config.retractSpeed + bonus);
}

void ClimberOuterReach::retract2() {
    double bonus = (mIsUnderLoad)
        ? config.friction.outerStaticFrictionWithLoad
        : config.friction.outerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor2.Set(config.retractSpeed + bonus);
}

void ClimberOuterReach::run1(double speed) {
    mMotor1.Set(speed);
}

void ClimberOuterReach::run2(double speed) {
    mMotor2.Set(speed);
}

void ClimberOuterReach::stop1() {
    mMotor1.StopMotor();
}

void ClimberOuterReach::stop2() {
    mMotor2.StopMotor();
}

double ClimberOuterReach::getMotor1Position() {
    return (mEncoder1.GetPosition() * config.inchesPerRevolution);
}

double ClimberOuterReach::getMotor2Position() {
    return (mEncoder2.GetPosition() * config.inchesPerRevolution);
}

bool ClimberOuterReach::isMotor1NearTarget(double target) {
    return std::abs(target - getMotor1Position()) < constants::climb::kAcceptablePositionError;
}

bool ClimberOuterReach::isMotor2NearTarget(double target) {
    return std::abs(target - getMotor2Position()) < constants::climb::kAcceptablePositionError;
}

void ClimberOuterReach::setUnderLoad(bool isUnderLoad) {
    mIsUnderLoad = isUnderLoad;
}