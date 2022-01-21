#include "intake/intake.h"

const double kDeploySpeedFactor = 0.1;

void Intake::extendIntake () {
    if (mDeployTargetSpeed != mDeployCurrentSpeed) { // add a tenth of the difference to current_speed
        mDeployCurrentSpeed += kDeploySpeedFactor * (mDeployTargetSpeed - mDeployCurrentSpeed);
    }
    mDeployMotor.Set(ControlMode::PercentOutput, mDeployCurrentSpeed);
}

void Intake::retractIntake () {
    if (-mDeployTargetSpeed != mDeployCurrentSpeed) { // add a tenth of the difference to current_speed
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
    return mIntakePosition.Get();
}

double Intake::getDistanceToPosition (double targetPosition) {
    return targetPosition - getIntakePosition();
}