#include "intake/intake.h"

Intake::Intake (std::shared_ptr<cpptoml::table> toml) {
    config.rollerSpeed = toml->get_qualified_as<double>("rollerSpeed").value_or(0.5);
    config.extendTargetSpeed = toml->get_qualified_as<double>("extendTargetSpeed").value_or(0.5);
    config.intakeExtendedPosition = toml->get_qualified_as<double>("intakeExtendedPosition").value_or(0.0);
    config.intakeRetractedPosition = toml->get_qualified_as<double>("intakeRetractedPosition").value_or(0.0);
}

void Intake::extendIntake () {
    mDeployServo1.Set(config.intakeExtendedPosition);
    mDeployServo2.Set(config.intakeExtendedPosition);
    mCurrentIntakeStatus = EXTENDING;
}

void Intake::retractIntake () {
    mDeployServo1.Set(config.intakeRetractedPosition);
    mDeployServo2.Set(config.intakeRetractedPosition);;
    mCurrentIntakeStatus = RETRACTING;
}

void Intake::runRollers () {
    mRollerMotor.Set(config.rollerSpeed);
}

void Intake::stopRollers () {
    mRollerMotor.Set(0.0);
}

bool Intake::isIntakeExtended () {
    return mIsIntakeExtended;
}

void Intake::setStationary (bool isExtended) {
    mCurrentIntakeStatus = STATIONARY;
    mIsIntakeExtended = isExtended;
}