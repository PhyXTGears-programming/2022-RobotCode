#include "climber/OuterRotate.h"

#include <frc/smartdashboard/SmartDashboard.h>

ClimberOuterRotate::ClimberOuterRotate(std::shared_ptr<cpptoml::table> toml) {
    mEncoder.SetPositionOffset(toml->get_qualified_as<double>("outerRotationZeroOffset").value_or(0.023));

    mMotorEncoder.SetPosition(mEncoder.GetAbsolutePosition());
}

void ClimberOuterRotate::Periodic() {
    frc::SmartDashboard::PutNumber("Clmb Out Angle", getAngle());

    frc::SmartDashboard::PutNumber("Clmb Out Rot Amps", mMotor.GetOutputCurrent());
}

void ClimberOuterRotate::rotate(double speed) {
    mMotor.Set(speed);
}

void ClimberOuterRotate::stop() {
    ClimberOuterRotate::rotate(0.0);
}

double ClimberOuterRotate::getAngle() {
    return mMotorEncoder.GetPosition();
}

void ClimberOuterRotate::setMotorCoast() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void ClimberOuterRotate::setMotorBrake() {
    mMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void ClimberOuterRotate::setCurrentlimit(unsigned int limit){
    mMotor.SetSmartCurrentLimit(limit);
}

void ClimberOuterRotate::resetCurrentLimit(){
    ClimberOuterRotate::setCurrentlimit(40);
}