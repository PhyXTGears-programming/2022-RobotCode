#include "intake/intake.h"

void Intake::runRollers (double speed) {
    mRollerMotor.Set(ControlMode::PercentOutput, speed);
}

void Intake::stopRollers () {
    runRollers(0.0);
}