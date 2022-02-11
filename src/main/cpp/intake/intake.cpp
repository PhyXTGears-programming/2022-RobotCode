#include "intake/intake.h"

Intake::Intake (std::shared_ptr<cpptoml::table> toml) {
    config.rollerSpeed = toml->get_qualified_as<double>("rollerSpeed").value_or(0.5);
    config.extendTargetSpeed = toml->get_qualified_as<double>("extendTargetSpeed").value_or(0.5);
    config.retractTargetSpeed = toml->get_qualified_as<double>("retractTargetSpeed").value_or(0.5);
    config.deploySpeedFactor = toml->get_qualified_as<double>("deploySpeedFactor").value_or(0.01);
    config.intakeExtendedPosition = toml->get_qualified_as<double>("intakeExtendedPosition").value_or(0.0);
    config.intakeRetractedPosition = toml->get_qualified_as<double>("intakeRetractedPosition").value_or(0.0);
}

void Intake::extendIntake () {
    mDeployTargetSpeed = config.extendTargetSpeed;
    mCurrentIntakeStatus = EXTENDING;
}

void Intake::retractIntake () {
    mDeployTargetSpeed = config.retractTargetSpeed;
    mCurrentIntakeStatus = RETRACTING;
}

void Intake::moveIntake () {
    if (mDeployTargetSpeed != mDeployCurrentSpeed) { // add a tenth of the difference to current_speed
        mDeployCurrentSpeed += config.deploySpeedFactor * (mDeployTargetSpeed - mDeployCurrentSpeed);
    }
    mDeployMotor.Set(ControlMode::PercentOutput, mDeployCurrentSpeed);
}

void Intake::runRollers () {
    mRollerMotor.Set(ControlMode::PercentOutput, config.rollerSpeed);
}

void Intake::stopRollers () {
    mRollerMotor.Set(ControlMode::PercentOutput, 0.0);
}

bool Intake::isIntakeExtended () {
    return mIsIntakeExtended;
}

double Intake::getIntakePosition () {
    return mIntakePosition.Get(); // mIntakePosition is a potentiometer. Get() gets its current position
} 

double Intake::getDistanceToPosition () {
    switch (mCurrentIntakeStatus) {
    case EXTENDING:
        return config.intakeExtendedPosition - getIntakePosition();
        break;
    
    case RETRACTING:
        return config.intakeRetractedPosition - getIntakePosition();
        break;
    
    default:
        return 0.0;
        break;
    }
}

void Intake::setStationary (bool isExtended) {
    mCurrentIntakeStatus = STATIONARY;
    mIsIntakeExtended = isExtended;
    mDeployTargetSpeed = 0.0;
}