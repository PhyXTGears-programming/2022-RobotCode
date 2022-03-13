#include "climber/OuterReach.h"

#include <cmath>

const double kAcceptablePositionError = 0.3;

OuterReach::OuterReach(std::shared_ptr<cpptoml::table> toml) {
    config.extendSpeed = toml->get_qualified_as<double>("extendSpeed").value_or(0.5);
    config.retractSpeed = toml->get_qualified_as<double>("retractSpeed").value_or(-0.5);
    config.inchesPerRevolution = toml->get_qualified_as<double>("inchesPerRevolution").value_or(0.128325);
    config.friction.innerStaticFriction = toml->get_qualified_as<double>("innerStaticFriction").value_or(0.0);
    config.friction.outerStaticFriction = toml->get_qualified_as<double>("outerStaticFriction").value_or(0.0);
    config.friction.innerStaticFrictionWithLoad = toml->get_qualified_as<double>("innerStaticFrictionWithLoad").value_or(0.0);
    config.friction.outerStaticFrictionWithLoad = toml->get_qualified_as<double>("outerStaticFrictionWithLoad").value_or(0.0);

    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mMotor1.SetInverted(true);
    mMotor2.SetInverted(false);

    mEncoder1.SetPosition(0.0);
    mEncoder2.SetPosition(0.0);
}

void OuterReach::extend1() {
    double bonus = std::copysign(config.friction.outerStaticFriction, config.extendSpeed);
    mMotor1.Set(config.extendSpeed + bonus);
}

void OuterReach::extend2() {
    double bonus = std::copysign(config.friction.outerStaticFriction, config.extendSpeed);
    mMotor2.Set(config.extendSpeed + bonus);
}

void OuterReach::retract1() {
    double bonus = (mIsUnderLoad)
        ? config.friction.outerStaticFrictionWithLoad
        : config.friction.outerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor1.Set(config.retractSpeed + bonus);
}

void OuterReach::retract2() {
    double bonus = (mIsUnderLoad)
        ? config.friction.outerStaticFrictionWithLoad
        : config.friction.outerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor2.Set(config.retractSpeed + bonus);
}

void OuterReach::stop1() {
    mMotor1.Set(0.0);
}

void OuterReach::stop2() {
    mMotor2.Set(0.0);
}

double OuterReach::getMotor1Position() {
    return (mEncoder1.GetPosition() * config.inchesPerRevolution);
}

double OuterReach::getMotor2Position() {
    return (mEncoder2.GetPosition() * config.inchesPerRevolution);
}

bool OuterReach::isMotor1NearTarget(double target) {
    return std::abs(target - getMotor1Position()) < kAcceptablePositionError;
}

bool OuterReach::isMotor2NearTarget(double target) {
    return std::abs(target - getMotor2Position()) < kAcceptablePositionError;
}

void OuterReach::setUnderLoad(bool isUnderLoad) {
    mIsUnderLoad = isUnderLoad;
}