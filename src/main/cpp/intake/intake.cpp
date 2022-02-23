#include "intake/intake.h"

Intake::Intake (std::shared_ptr<cpptoml::table> toml) {
    config.rollerSpeed = toml->get_qualified_as<double>("rollerSpeed").value_or(0.5);
}

void Intake::runRollers () {
    mRollerMotor.Set(config.rollerSpeed);
}

void Intake::stopRollers () {
    mRollerMotor.Set(0.0);
}