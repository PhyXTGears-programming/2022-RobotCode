#include "commands/RetractIntake.h"

const double kAcceptablePositionError = 0.01; // buffer value for potentiometer position

RetractIntakeCommand::RetractIntakeCommand (Intake* intake) {
    AddRequirements(intake);
    mIntake = intake;
}

void RetractIntakeCommand::Initialize () {
    mIntake->retractIntake();
}

void RetractIntakeCommand::Execute () {
    mIntake->moveIntake();
}

void RetractIntakeCommand::End (bool interrupted) {
    mIntake->setStationary(false);
}

bool RetractIntakeCommand::IsFinished () {
    return abs(mIntake->getDistanceToPosition()) < kAcceptablePositionError;
}