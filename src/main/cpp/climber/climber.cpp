#include "climber/climber.h"

const double kAcceptablePositionError = 0.05;

Climber::Climber(std::shared_ptr<cpptoml::table> toml) {
    config.lockServoPosition = toml->get_qualified_as<double>("lockServoPosition").value_or(0.0);
    config.extendSpeed = toml->get_qualified_as<double>("extendSpeed").value_or(0.5);
    config.retractSpeed = toml->get_qualified_as<double>("retractSpeed").value_or(-0.5);
    config.inchesPerRevolution = toml->get_qualified_as<double>("inchesPerRevolution").value_or(0.128325);
    config.friction.innerStaticFriction = toml->get_qualified_as<double>("innerStaticFriction").value_or(0.0);
    config.friction.outerStaticFriction = toml->get_qualified_as<double>("outerStaticFriction").value_or(0.0);
    config.friction.innerStaticFrictionWithLoad = toml->get_qualified_as<double>("innerStaticFrictionWithLoad").value_or(0.0);
    config.friction.outerStaticFrictionWithLoad = toml->get_qualified_as<double>("outerStaticFrictionWithLoad").value_or(0.0);

    mInnerHookMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mInnerHookMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mOuterHookMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mOuterHookMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mOuterHook1Encoder.SetPosition(0.0);
    mOuterHook2Encoder.SetPosition(0.0);
    mInnerHook1Encoder.SetPosition(0.0);
    mInnerHook2Encoder.SetPosition(0.0);
}

void Climber::extendOuter1() {
    mOuterHookMotor1.Set(config.extendSpeed + ((mIsUnderLoad) ? config.friction.outerStaticFrictionWithLoad : config.friction.outerStaticFriction));
}

void Climber::extendOuter2() {
    mOuterHookMotor2.Set(config.extendSpeed + ((mIsUnderLoad) ? config.friction.outerStaticFrictionWithLoad : config.friction.outerStaticFriction));
}

void Climber::retractOuter1() {
    mOuterHookMotor1.Set(config.retractSpeed + config.friction.outerStaticFriction);
}

void Climber::retractOuter2() {
    mOuterHookMotor2.Set(config.retractSpeed + config.friction.outerStaticFriction);
}

void Climber::stopOuter1() {
    mOuterHookMotor1.Set(0.0);
}

void Climber::stopOuter2() {
    mOuterHookMotor2.Set(0.0);
}

void Climber::extendInner1() {
    mInnerHookMotor1.Set(config.extendSpeed + ((mIsUnderLoad) ? config.friction.innerStaticFrictionWithLoad : config.friction.innerStaticFriction));
}

void Climber::extendInner2() {
    mInnerHookMotor2.Set(config.extendSpeed + ((mIsUnderLoad) ? config.friction.innerStaticFrictionWithLoad : config.friction.innerStaticFriction));
}

void Climber::retractInner1() {
    mOuterHookMotor1.Set(config.retractSpeed + config.friction.innerStaticFriction);
}
    
void Climber::retractInner2() {
    mOuterHookMotor2.Set(config.retractSpeed + config.friction.innerStaticFriction);
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

void Climber::rotateInner(double speed) {
    mInnerArmRotationMotor.Set(speed);
}

void Climber::rotateOuter(double speed) {
    mOuterArmRotationMotor.Set(speed);
}

double Climber::getInnerAngle() {
    return mInnerRotationEncoder.GetAbsolutePosition();
}

double Climber::getOuterAngle() {
    return mOuterRotationEncoder.GetAbsolutePosition();
}


void Climber::setUnderLoad(bool isUnderLoad) {
    mIsUnderLoad = isUnderLoad;
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