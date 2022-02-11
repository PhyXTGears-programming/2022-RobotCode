#include "commands/ExtendIntake.h"

const double kAcceptablePositionError = 0.01; // buffer value for potentiometer position

ExtendIntakeCommand::ExtendIntakeCommand (Intake* intake) {
    AddRequirements(intake);
    mIntake = intake;
}

void ExtendIntakeCommand::Initialize () {
    mIntake->extendIntake();
}

void ExtendIntakeCommand::Execute () {
    mIntake->moveIntake();
}

void ExtendIntakeCommand::End (bool interrupted) {
    mIntake->setStationary(true); // the function is setStationary (isExtended)
}

bool ExtendIntakeCommand::IsFinished () {
    return abs(mIntake->getDistanceToPosition()) < kAcceptablePositionError;
}