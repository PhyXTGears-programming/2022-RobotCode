#include "climber/climber.h"

#include <cmath>

Climber::OuterReach::OuterReach() {
    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mMotor1.SetInverted(true);
    mMotor2.SetInverted(false);

    mEncoder1.SetPosition(0.0);
    mEncoder2.SetPosition(0.0);
}

void Climber::OuterReach::extend1() {
    double bonus = std::copysign(config.friction.outerStaticFriction, config.extendSpeed);
    mMotor1.Set(config.extendSpeed);
}

void Climber::OuterReach::extend2() {
    double bonus = std::copysign(config.friction.outerStaticFriction, config.extendSpeed);
    mMotor2.Set(config.extendSpeed + bonus);
}

void Climber::OuterReach::retract1() {
    double bonus = (mIsUnderLoad)
        ? config.friction.outerStaticFrictionWithLoad
        : config.friction.outerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor1.Set(config.retractSpeed + bonus);
}

void Climber::OuterReach::retract2() {
    double bonus = (mIsUnderLoad)
        ? config.friction.outerStaticFrictionWithLoad
        : config.friction.outerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor2.Set(config.retractSpeed + bonus);
}

void Climber::OuterReach::stop1() {
    mMotor1.Set(0.0);
}

void Climber::OuterReach::stop2() {
    mMotor2.Set(0.0);
}

double Climber::OuterReach::getMotor1Position() {
    return (mEncoder1.GetPosition() * config.inchesPerRevolution);
}

double Climber::OuterReach::getMotor2Position() {
    return (mEncoder2.GetPosition() * config.inchesPerRevolution);
}

bool Climber::OuterReach::isMotor1NearTarget(double target) {
    return std::abs(target - getMotor1Position()) < kAcceptablePositionError;
}

bool Climber::OuterReach::isMotor2NearTarget(double target) {
    return std::abs(target - getMotor2Position()) < kAcceptablePositionError;
}

void Climber::OuterReach::setUnderLoad(bool isUnderLoad) {
    mIsUnderLoad = isUnderLoad;
}