#include "intake/intake.h"

const double kDeploySpeedFactor = 0.1;

void Intake::extendIntake () {
    mDeployTargetSpeed = kExtendTargetSpeed;
    mCurrentIntakeStatus = EXTENDING;
}

void Intake::retractIntake () {
    mDeployTargetSpeed = kRetractTargetSpeed;
    mCurrentIntakeStatus = RETRACTING;
}

void Intake::moveIntake () {
    if (mDeployTargetSpeed != mDeployCurrentSpeed) { // add a tenth of the difference to current_speed
        mDeployCurrentSpeed += kDeploySpeedFactor * (mDeployTargetSpeed - mDeployCurrentSpeed);
    }
    mDeployMotor.Set(ControlMode::PercentOutput, mDeployCurrentSpeed);
}

void Intake::runRollers (double speed) {
    mRollerMotor.Set(ControlMode::PercentOutput, speed);
}

void Intake::stopRollers () {
    runRollers(0.0);
}

bool Intake::isIntakeExtended () {
    return mIsIntakeExtended;
}

double Intake::getIntakePosition () {
    return mIntakePosition.Get(); // mIntakePosition is a potentiometer. Get() gets its current position
} 

double Intake::getDistanceToPosition () {
    switch (mCurrentIntakeStatus)
    {
    case EXTENDING:
        return mIntakeExtendedPosition - getIntakePosition();
        break;
    
    case RETRACTING:
        return mIntakeRetractedPosition - getIntakePosition();
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