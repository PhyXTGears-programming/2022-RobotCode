#include "commands/climber/RotateOuterArms.h"
#include "constants/constants.h"

#include <cmath>

RotateOuterArmsCommand::RotateOuterArmsCommand(ClimberOuterRotate * outerArms, double targetAngle, PID & pid) {
    AddRequirements(outerArms);
    mOuterArms = outerArms;
    mTargetAngle = targetAngle;
    mPid = pid;
}

void RotateOuterArmsCommand::Initialize() {
    mOuterArms->setMotorBrake();
    mPid.setTarget(mTargetAngle);
}

void RotateOuterArmsCommand::Execute() {
    // Angles are not radians or degrees.  They are duty cycle values...
    // somewhere between -1.0 and 1.0 it seems.  So use error to set direction
    // of rotation, (+) is lean forward, (-) is lean backward.
    double armAngle = mOuterArms->getAngle();
    
    mOuterArms->rotate(mPid.calculate(armAngle));
}

void RotateOuterArmsCommand::End(bool isInterrupted) {
    mOuterArms->stop();
}

bool RotateOuterArmsCommand::IsFinished() {
    return std::abs(mPid.getError())         < constants::climb::kAcceptableAngleError
        && std::abs(mPid.getVelocityError()) < constants::climb::kAcceptableVelocityError;
}
