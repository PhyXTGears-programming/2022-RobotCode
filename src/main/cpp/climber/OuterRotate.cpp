#include "climber/OuterRotate.h"

#include <frc/smartdashboard/SmartDashboard.h>

ClimberOuterRotate::ClimberOuterRotate(std::shared_ptr<cpptoml::table> toml) {
    mEncoder.SetPositionOffset(toml->get_qualified_as<double>("outerRotationZeroOffset").value_or(0.023));
}

void ClimberOuterRotate::Periodic() {
    frc::SmartDashboard::PutNumber("Clmb Out Angle", getAngle());
}

void ClimberOuterRotate::rotate(double speed) {
    mMotor.Set(speed);
}

void ClimberOuterRotate::stop() {
    ClimberOuterRotate::rotate(0.0);
}

double ClimberOuterRotate::getAngle() {
    return mEncoder.GetAbsolutePosition();
}

void ClimberOuterRotate::setMotorCoast() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void ClimberOuterRotate::setMotorBrake() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}