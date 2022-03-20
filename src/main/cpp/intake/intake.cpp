#include "intake/intake.h"

Intake::Intake (std::shared_ptr<cpptoml::table> toml) {
    config.rollerSpeed = toml->get_qualified_as<double>("rollerSpeed").value_or(0.5);
    
    mRollerMotor.SetInverted(true);
}

void Intake::runRollers () {
    mRollerMotor.Set(config.rollerSpeed);
}

void Intake::runRollersReverse () {
    mRollerMotor.Set(-config.rollerSpeed);
}

void Intake::stopRollers () {
    mRollerMotor.Set(0.0);
}

void Intake::extend () {
    mRetractRight.Set(false);
    mRetractLeft.Set(false);

    mExtendRight.Set(true);
    mExtendLeft.Set(true);

    mIsExtended = true;
}

void Intake::retract () {
    mExtendRight.Set(false);
    mExtendLeft.Set(false);
    
    mRetractRight.Set(true);
    mRetractLeft.Set(true);

    mIsExtended = false;
}

bool Intake::isExtended () {
    return mIsExtended;
}