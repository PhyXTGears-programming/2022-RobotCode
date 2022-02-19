#include "climber/climber.h"

Climber::Climber(std::shared_ptr<cpptoml::table> toml) {
    config.lockServoPosition = toml->get_qualified_as<double>("lockServoPosition").value_or(0.0);
    config.extendSpeed = toml->get_qualified_as<double>("extendSpeed").value_or(0.0);
    config.retractSpeed = toml->get_qualified_as<double>("retractSpeed").value_or(0.0);
    config.friction.innerStaticFriction = toml->get_qualified_as<double>("innerStaticFriction").value_or(0.0);
    config.friction.outerStaticFriction = toml->get_qualified_as<double>("outerStaticFriction").value_or(0.0);
    config.friction.innerStaticFrictionWithLoad = toml->get_qualified_as<double>("innerStaticFrictionWithLoad").value_or(0.0);
    config.friction.outerStaticFrictionWithLoad = toml->get_qualified_as<double>("outerStaticFrictionWithLoad").value_or(0.0);
}

void Climber::extendOuter() {
    mOuterHookMotor1.Set(config.extendSpeed);
    mOuterHookMotor2.Set(config.extendSpeed);
}

void Climber::retractOuter() {
    mOuterHookMotor1.Set(config.retractSpeed);
    mOuterHookMotor2.Set(config.retractSpeed);
}

void Climber::stopOuter() {
    mOuterHookMotor1.Set(0.0);
    mOuterHookMotor2.Set(0.0);
}

void Climber::extendInner() {
    mInnerHookMotor1.Set(config.extendSpeed);
    mInnerHookMotor2.Set(config.extendSpeed);
}

void Climber::retractInner() {
    mOuterHookMotor1.Set(config.retractSpeed);
    mOuterHookMotor2.Set(config.retractSpeed);
}

void Climber::stopInner() {
    mInnerHookMotor1.Set(0.0);
    mInnerHookMotor2.Set(0.0);
}

void Climber::lockArms() {
    mStopServo1.Set(config.lockServoPosition);
    mStopServo2.Set(config.lockServoPosition);
}

void Climber::updateRelay(bool isOn) {
    if (isOn) {
        mBackDriveRelay.Set(frc::Relay::Value::kOn);
    } else {
        mBackDriveRelay.Set(frc::Relay::Value::kOff);
    }
}

void Climber::rotateInner(double targetPosition) {
    mInnerServo1.Set(targetPosition);
    mInnerServo2.Set(targetPosition);
}

void Climber::rotateOuter(double targetPosition) {
    mOuterServo1.Set(targetPosition);
    mOuterServo2.Set(targetPosition);
}

void Climber::setUnderLoad(bool isUnderLoad) {
    mIsUnderLoad = isUnderLoad;
}

double Climber::getInnerArmRotationsFromTarget(double targetRotations) {
    return targetRotations - (mInnerHook1Encoder.GetPosition() + mInnerHook2Encoder.GetPosition()) / 2;
}

double Climber::getOuterArmRotationsFromTarget(double targetRotations) {
    return targetRotations - (mOuterHook1Encoder.GetPosition() + mOuterHook2Encoder.GetPosition()) / 2;
}