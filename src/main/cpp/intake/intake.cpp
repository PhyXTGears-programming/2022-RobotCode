#include "intake/intake.h"

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