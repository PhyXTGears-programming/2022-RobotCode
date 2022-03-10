#include "climber/climber.h"

#include <cmath>

Climber::InnerReach::InnerReach() {
    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mMotor1.SetInverted(true);
    mMotor2.SetInverted(false);

    mEncoder1.SetPosition(0.0);
    mEncoder2.SetPosition(0.0);
}

void Climber::InnerReach::extend1() {
    double bonus = std::copysign(config.friction.innerStaticFriction, config.extendSpeed);
    mMotor1.Set(config.extendSpeed + bonus);
}

void Climber::InnerReach::extend2() {
    double bonus = std::copysign(config.friction.innerStaticFriction, config.extendSpeed);
    mMotor2.Set(config.extendSpeed + bonus);
}

void Climber::InnerReach::retract1() {
    double bonus = (mIsUnderLoad)
        ? config.friction.innerStaticFrictionWithLoad
        : config.friction.innerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor1.Set(config.retractSpeed + bonus);
}
    
void Climber::InnerReach::retract2() {
    double bonus = (mIsUnderLoad)
        ? config.friction.innerStaticFrictionWithLoad
        : config.friction.innerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor2.Set(config.retractSpeed + bonus);
}

void Climber::InnerReach::stop1() {
    mMotor1.Set(0.0);
}

void Climber::InnerReach::stop2() {
    mMotor2.Set(0.0);
}

void Climber::InnerReach::run1(double speed) {
    mMotor1.Set(speed);
}

void Climber::InnerReach::run2(double speed) {
    mMotor2.Set(speed);
}

double Climber::InnerReach::getMotor1Position() {
    return (mEncoder1.GetPosition() * config.inchesPerRevolution);
}

double Climber::InnerReach::getMotor2Position() {
    return (mEncoder2.GetPosition() * config.inchesPerRevolution);
}

bool Climber::InnerReach::isMotor1NearTarget(double target) {
    return std::abs(target - getMotor1Position()) < kAcceptablePositionError;
}

bool Climber::InnerReach::isMotor2NearTarget(double target) {
    return std::abs(target - getMotor2Position()) < kAcceptablePositionError;
}

void Climber::InnerReach::setMotorsCoast() {
    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Climber::InnerReach::setMotorsBrake() {
    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Climber::InnerReach::setUnderLoad(bool isUnderLoad) {
    mIsUnderLoad = isUnderLoad;
}

void Climber::InnerReach::lockArms() {
    mStopServo1.Set(config.servo1.lockPosition);
    mStopServo2.Set(config.servo2.lockPosition);
}

void Climber::InnerReach::unlockArms() {
    mStopServo1.Set(config.servo1.unlockPosition);
    mStopServo2.Set(config.servo2.unlockPosition);
}