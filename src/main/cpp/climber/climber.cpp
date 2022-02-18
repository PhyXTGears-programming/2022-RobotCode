#include "climber/climber.h"

Climber::Climber(std::shared_ptr<cpptoml::table> toml) {
    config.lockServoPosition = toml->get_qualified_as<double>("lockServoPosition").value_or(0.0);
    config.extendSpeed = toml->get_qualified_as<double>("extendSpeed").value_or(0.0);
    config.retractSpeed = toml->get_qualified_as<double>("retractSpeed").value_or(0.0);
    config.friction.innerStaticFriction = toml->get_qualified_as<double>("innerStaticFriction").value_or(0.0);
    config.friction.outerStaticFriction = toml->get_qualified_as<double>("outerStaticFriction").value_or(0.0);
    config.friction.innerStaticFrictionWithLoad = toml->get_qualified_as<double>("innerStaticFrictionWithLoad").value_or(0.0);
}

void Climber::extendOuter() {
    mOuterHookMotor1.Set(config.extendSpeed);
    mOuterHookMotor2.Set(config.extendSpeed);
}

void Climber::retractOuter() {
    mOuterHookMotor1.Set(config.retractSpeed);
    mOuterHookMotor2.Set(config.retractSpeed);
}

void Climber::extendInner() {
    mInnerHookMotor1.Set(config.extendSpeed);
    mInnerHookMotor2.Set(config.extendSpeed);
}

void Climber::retractInner() {
    mOuterHookMotor1.Set(config.retractSpeed);
    mOuterHookMotor2.Set(config.retractSpeed);
}

void Climber::lockArms() {
    mStopServo1.set(config.lockServoPosition);
    mStopServo2.set(config.lockServoPosition);
}

void Climber::updateRelay(bool on) {
    if (on) {
        mBackDriveRelay.set(frc::Relay.Value.kOn);
    } else {
        mBackDriveRelay.set(frc::Relay.Value.kOff);
    }
}

double Climber::getInnerArmRotations() { // returns average of the hook positions
    return (mInnerHook1Encoder.GetPosition() + mInnerHook2Encoder.GetPosition())/2;
}

double Climber::getOuterArmRotations() { // returns average of the hook positions
    return (mOuterHook1Encoder.GetPosition() + mOuterHook2Encoder.GetPosition())/2;
}