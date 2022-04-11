#include "commands/climber/RotateInnerArms.h"
#include "constants/constants.h"

#include <cmath>

RotateInnerArmsCommand::RotateInnerArmsCommand(ClimberInnerRotate * innerArms, double targetAngle, PID & pid) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
    mTargetAngle = targetAngle;
    mPid = pid;
}

void RotateInnerArmsCommand::Initialize() {
    mInnerArms->setMotorBrake();
    mPid.setTarget(mTargetAngle);
}

void RotateInnerArmsCommand::Execute() {
    // Angles are not radians or degrees.  They are duty cycle values...
    // somewhere between -1.0 and 1.0 it seems.  So use error to set direction
    // of rotation, (+) is lean forward, (-) is lean backward.
    double armAngle = mInnerArms->getAngle();

    mInnerArms->rotate(mPid.calculate(armAngle));
}

void RotateInnerArmsCommand::End(bool isInterrupted) {
    mInnerArms->stop();
}

bool RotateInnerArmsCommand::IsFinished() {
    return std::abs(mPid.getError())         < constants::climb::kAcceptableAngleError
        && std::abs(mPid.getVelocityError()) < constants::climb::kAcceptableVelocityError;
}
