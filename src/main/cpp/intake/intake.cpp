#include "intake/intake.h"

Intake::Intake (std::shared_ptr<cpptoml::table> toml) {
    config.rollerSpeed = toml->get_qualified_as<double>("rollerSpeed").value_or(0.5);
    
    mRollerMotor.SetInverted(true);
}

void Intake::runRollers () {
    mRollerMotor.Set(config.rollerSpeed);
}

void Intake::stopRollers () {
    mRollerMotor.Set(0.0);
}

void Intake::extend () {
    mRetract.Set(false);
    mExtend.Set(true);
}

void Intake::retract () {
    mExtend.Set(false);
    mRetract.Set(true);
}

bool Intake::isExtended () {
    return mIsExtended;
}