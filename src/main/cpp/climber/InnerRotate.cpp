#include "climber/climber.h"

Climber::InnerRotate::InnerRotate() {
    mEncoder.SetPositionOffset(0.962);
    //mInnerRotationEncoder.SetPositionOffset(toml->get_qualified_as<double>("outerPositionOffset").value_or(0.962));
}

void Climber::InnerRotate::rotate(double speed) {
    mMotor.Set(speed);
}

double Climber::InnerRotate::getAngle() {
    return mEncoder.GetAbsolutePosition();
}

void Climber::InnerRotate::setMotorCoast() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Climber::InnerRotate::setMotorBrake() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}
