#include "climber/climber.h"

const double kAcceptablePositionError = 0.05;

Climber::Climber(std::shared_ptr<cpptoml::table> toml) {
    config.lockServoPosition = toml->get_qualified_as<double>("lockServoPosition").value_or(0.0);
    config.extendSpeed = toml->get_qualified_as<double>("extendSpeed").value_or(0.5);
    config.retractSpeed = toml->get_qualified_as<double>("retractSpeed").value_or(0.5);
    config.inchesPerRevolution = toml->get_qualified_as<double>("inchesPerRevolution").value_or(0.128325);
    config.friction.innerStaticFriction = toml->get_qualified_as<double>("innerStaticFriction").value_or(0.0);
    config.friction.outerStaticFriction = toml->get_qualified_as<double>("outerStaticFriction").value_or(0.0);
    config.friction.innerStaticFrictionWithLoad = toml->get_qualified_as<double>("innerStaticFrictionWithLoad").value_or(0.0);
    config.friction.outerStaticFrictionWithLoad = toml->get_qualified_as<double>("outerStaticFrictionWithLoad").value_or(0.0);
}

void Climber::extendOuter1() {
    mOuterHookMotor1.Set(config.extendSpeed);
}

void Climber::extendOuter2() {
    mOuterHookMotor2.Set(config.extendSpeed);
}

void Climber::retractOuter1() {
    mOuterHookMotor1.Set(config.retractSpeed);
}

void Climber::retractOuter2() {
    mOuterHookMotor2.Set(config.retractSpeed);
}

void Climber::stopOuter1() {
    mOuterHookMotor1.Set(0.0);
}

void Climber::stopOuter2() {
    mOuterHookMotor2.Set(0.0);
}

void Climber::extendInner1() {
    mInnerHookMotor1.Set(config.extendSpeed);
}

void Climber::extendInner2() {
    mInnerHookMotor2.Set(config.extendSpeed);
}

void Climber::retractInner1() {
    mOuterHookMotor1.Set(config.retractSpeed);
}
    
void Climber::retractInner2() {
    mOuterHookMotor2.Set(config.retractSpeed);
}

void Climber::stopInner1() {
    mInnerHookMotor1.Set(0.0);
}

void Climber::stopInner2() {
    mInnerHookMotor2.Set(0.0);
}

void Climber::lockArms() {
    mStopServo1.Set(config.lockServoPosition);
    mStopServo2.Set(config.lockServoPosition);
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

void Climber::disableServos() {
    mInnerServo1.SetOffline();
    mInnerServo2.SetOffline();
    mOuterServo1.SetOffline();
    mOuterServo2.SetOffline();
}

bool Climber::isOuter1NearTarget(double target) {
    return abs(target - (mOuterHook1Encoder.GetPosition() * config.inchesPerRevolution)) < kAcceptablePositionError;
}

bool Climber::isOuter2NearTarget(double target) {
    return abs(target - (mOuterHook2Encoder.GetPosition() * config.inchesPerRevolution)) < kAcceptablePositionError;
}

bool Climber::isInner1NearTarget(double target) {
    return abs(target - (mInnerHook1Encoder.GetPosition() * config.inchesPerRevolution)) < kAcceptablePositionError;
}

bool Climber::isInner2NearTarget(double target) {
    return abs(target - (mInnerHook2Encoder.GetPosition() * config.inchesPerRevolution)) < kAcceptablePositionError;
}