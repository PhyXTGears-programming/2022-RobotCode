#include "climber/InnerReach.h"

#include <cmath>

const double kAcceptablePositionError = 0.3;

InnerReach::InnerReach(std::shared_ptr<cpptoml::table> toml) {
    config.servo1.unlockPosition = toml->get_qualified_as<double>("servo1.unlockPosition").value_or(0.0);
    config.servo1.lockPosition = toml->get_qualified_as<double>("servo1.lockPosition").value_or(0.0);

    config.servo2.unlockPosition = toml->get_qualified_as<double>("servo2.unlockPosition").value_or(0.0);
    config.servo2.lockPosition = toml->get_qualified_as<double>("servo2.lockPosition").value_or(0.0);

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

void InnerReach::extend1() {
    double bonus = std::copysign(config.friction.innerStaticFriction, config.extendSpeed);
    mMotor1.Set(config.extendSpeed + bonus);
}

void InnerReach::extend2() {
    double bonus = std::copysign(config.friction.innerStaticFriction, config.extendSpeed);
    mMotor2.Set(config.extendSpeed + bonus);
}

void InnerReach::retract1() {
    double bonus = (mIsUnderLoad)
        ? config.friction.innerStaticFrictionWithLoad
        : config.friction.innerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor1.Set(config.retractSpeed + bonus);
}
    
void InnerReach::retract2() {
    double bonus = (mIsUnderLoad)
        ? config.friction.innerStaticFrictionWithLoad
        : config.friction.innerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor2.Set(config.retractSpeed + bonus);
}

void InnerReach::stop1() {
    mMotor1.Set(0.0);
}

void InnerReach::stop2() {
    mMotor2.Set(0.0);
}

void InnerReach::run1(double speed) {
    mMotor1.Set(speed);
}

void InnerReach::run2(double speed) {
    mMotor2.Set(speed);
}

double InnerReach::getMotor1Position() {
    return (mEncoder1.GetPosition() * config.inchesPerRevolution);
}

double InnerReach::getMotor2Position() {
    return (mEncoder2.GetPosition() * config.inchesPerRevolution);
}

bool InnerReach::isMotor1NearTarget(double target) {
    return std::abs(target - getMotor1Position()) < kAcceptablePositionError;
}

bool InnerReach::isMotor2NearTarget(double target) {
    return std::abs(target - getMotor2Position()) < kAcceptablePositionError;
}

void InnerReach::setMotorsCoast() {
    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void InnerReach::setMotorsBrake() {
    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void InnerReach::setUnderLoad(bool isUnderLoad) {
    mIsUnderLoad = isUnderLoad;
}

void InnerReach::lockArms() {
    mStopServo1.Set(config.servo1.lockPosition);
    mStopServo2.Set(config.servo2.lockPosition);
}

void InnerReach::unlockArms() {
    mStopServo1.Set(config.servo1.unlockPosition);
    mStopServo2.Set(config.servo2.unlockPosition);
}