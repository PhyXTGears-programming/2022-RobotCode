#include "climber/InnerRotate.h"

InnerRotate::InnerRotate(std::shared_ptr<cpptoml::table> toml) {
    mEncoder.SetPositionOffset(toml->get_qualified_as<double>("innerRotationZeroOffset").value_or(0.962));
}

void InnerRotate::rotate(double speed) {
    mMotor.Set(speed);
}

void InnerRotate::stop() {
    rotate(0.0);
}

double InnerRotate::getAngle() {
    return mEncoder.GetAbsolutePosition();
}

void InnerRotate::setMotorCoast() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void InnerRotate::setMotorBrake() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}
