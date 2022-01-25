#include "intake/intake.h"

const double kDeploySpeedFactor = 0.1;

void Intake::extendIntake () {
    if (mDeployTargetSpeed != mDeployCurrentSpeed) { // add a tenth of the difference to current_speed
        mDeployCurrentSpeed += kDeploySpeedFactor * (mDeployTargetSpeed - mDeployCurrentSpeed);
    }
    mDeployMotor.Set(ControlMode::PercentOutput, mDeployCurrentSpeed);

    mCurrentIntakeStatus = extending;
}

void Intake::retractIntake () {
    if (-mDeployTargetSpeed != mDeployCurrentSpeed) { // add a tenth of the difference to current_speed
        mDeployCurrentSpeed += kDeploySpeedFactor * (mDeployTargetSpeed - mDeployCurrentSpeed);
    }
    mDeployMotor.Set(ControlMode::PercentOutput, mDeployCurrentSpeed);

    mCurrentIntakeStatus = retracting;
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
    return mIntakePosition.Get();
}

double Intake::getDistanceToPosition () {
    switch (mCurrentIntakeStatus)
    {
    case extending:
        return mIntakeExtendedPosition - getIntakePosition();
        break;
    
    case retracting:
        return mIntakeRetractedPosition - getIntakePosition();
        break;
    
    default:
        return 0.0;
        break;
    }
}

void Intake::setStationary (bool isExtended) {
    mCurrentIntakeStatus = stationary;
    mIsIntakeExtended = isExtended;
}