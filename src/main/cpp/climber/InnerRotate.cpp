#include "climber/InnerRotate.h"

#include <frc/smartdashboard/SmartDashboard.h>

ClimberInnerRotate::ClimberInnerRotate(std::shared_ptr<cpptoml::table> toml) {
    mEncoder.SetPositionOffset(toml->get_qualified_as<double>("innerRotationZeroOffset").value_or(0.962));
}

void ClimberInnerRotate::Periodic() {
    frc::SmartDashboard::PutNumber("Clmb In Angle", getAngle());
}

void ClimberInnerRotate::rotate(double speed) {
    mMotor.Set(speed);
}

void ClimberInnerRotate::stop() {
    ClimberInnerRotate::rotate(0.0);
}

double ClimberInnerRotate::getAngle() {
    return mEncoder.GetAbsolutePosition();
}

void ClimberInnerRotate::setMotorCoast() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void ClimberInnerRotate::setMotorBrake() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}
