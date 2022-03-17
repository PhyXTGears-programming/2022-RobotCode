#include "climber/InnerReach.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>

ClimberInnerReach::ClimberInnerReach(std::shared_ptr<cpptoml::table> toml) {
    // config.servo1.unlockPosition = toml->get_qualified_as<double>("servo1.unlockPosition").value_or(0.0);
    // config.servo1.lockPosition = toml->get_qualified_as<double>("servo1.lockPosition").value_or(0.0);

    // config.servo2.unlockPosition = toml->get_qualified_as<double>("servo2.unlockPosition").value_or(0.0);
    // config.servo2.lockPosition = toml->get_qualified_as<double>("servo2.lockPosition").value_or(0.0);

    config.extendSpeed = toml->get_qualified_as<double>("extendSpeed").value_or(0.5);
    config.retractSpeed = toml->get_qualified_as<double>("retractSpeed").value_or(-0.5);
    config.inchesPerRevolution = toml->get_qualified_as<double>("inchesPerRevolution").value_or(0.128325);
    config.friction.innerStaticFriction = toml->get_qualified_as<double>("innerStaticFriction").value_or(0.0);
    config.friction.innerStaticFrictionWithLoad = toml->get_qualified_as<double>("innerStaticFrictionWithLoad").value_or(0.0);

    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mMotor1.SetInverted(true);
    mMotor2.SetInverted(false);

    mEncoder1.SetPosition(0.0);
    mEncoder2.SetPosition(0.0);
}

void ClimberInnerReach::Periodic() {
    frc::SmartDashboard::PutNumber("Clmb In R Pos", getMotor1Position());
    frc::SmartDashboard::PutNumber("Clmb In L Pos", getMotor2Position());
}

void ClimberInnerReach::extend1() {
    double bonus = std::copysign(config.friction.innerStaticFriction, config.extendSpeed);
    mMotor1.Set(config.extendSpeed + bonus);
}

void ClimberInnerReach::extend2() {
    double bonus = std::copysign(config.friction.innerStaticFriction, config.extendSpeed);
    mMotor2.Set(config.extendSpeed + bonus);
}

void ClimberInnerReach::retract1() {
    double bonus = (mIsUnderLoad)
        ? config.friction.innerStaticFrictionWithLoad
        : config.friction.innerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor1.Set(config.retractSpeed + bonus);
}
    
void ClimberInnerReach::retract2() {
    double bonus = (mIsUnderLoad)
        ? config.friction.innerStaticFrictionWithLoad
        : config.friction.innerStaticFriction;

    bonus = std::copysign(bonus, config.retractSpeed);

    mMotor2.Set(config.retractSpeed + bonus);
}

void ClimberInnerReach::stop1() {
    mMotor1.StopMotor();
}

void ClimberInnerReach::stop2() {
    mMotor2.StopMotor();
}

void ClimberInnerReach::run1(double speed) {
    mMotor1.Set(speed);
}

void ClimberInnerReach::run2(double speed) {
    mMotor2.Set(speed);
}

double ClimberInnerReach::getMotor1Position() {
    return (mEncoder1.GetPosition() * config.inchesPerRevolution);
}

double ClimberInnerReach::getMotor2Position() {
    return (mEncoder2.GetPosition() * config.inchesPerRevolution);
}

bool ClimberInnerReach::isMotor1NearTarget(double target) {
    return std::abs(target - getMotor1Position()) < constants::climb::kAcceptablePositionError;
}

bool ClimberInnerReach::isMotor2NearTarget(double target) {
    return std::abs(target - getMotor2Position()) < constants::climb::kAcceptablePositionError;
}

void ClimberInnerReach::setMotorsCoast() {
    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void ClimberInnerReach::setMotorsBrake() {
    mMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void ClimberInnerReach::setUnderLoad(bool isUnderLoad) {
    mIsUnderLoad = isUnderLoad;
}

// void ClimberInnerReach::lockArms() {
//     mStopServo1.Set(config.servo1.lockPosition);
//     mStopServo2.Set(config.servo2.lockPosition);
// }

// void ClimberInnerReach::unlockArms() {
//     mStopServo1.Set(config.servo1.unlockPosition);
//     mStopServo2.Set(config.servo2.unlockPosition);
// }