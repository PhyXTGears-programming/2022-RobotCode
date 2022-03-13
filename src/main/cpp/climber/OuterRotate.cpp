#include "climber/OuterRotate.h"

OuterRotate::OuterRotate(std::shared_ptr<cpptoml::table> toml) {
    mEncoder.SetPositionOffset(toml->get_qualified_as<double>("outerPositionOffset").value_or(0.023));
}

void OuterRotate::rotate(double speed) {
    mMotor.Set(speed);
}

double OuterRotate::getAngle() {
    return mEncoder.GetAbsolutePosition();
}

void OuterRotate::setMotorCoast() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void OuterRotate::setMotorBrake() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}