#include "climber/climber.h"

Climber::OuterRotate::OuterRotate() {
    mEncoder.SetPositionOffset(0.023);
    //mOuterRotationEncoder.SetPositionOffset(toml->get_qualified_as<double>("outerPositionOffset").value_or(0.023));
}

void Climber::OuterRotate::rotate(double speed) {
    mMotor.Set(speed);
}

double Climber::OuterRotate::getAngle() {
    return mEncoder.GetAbsolutePosition();
}

void Climber::OuterRotate::setMotorCoast() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Climber::OuterRotate::setMotorBrake() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}