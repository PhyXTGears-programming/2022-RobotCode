#include "commands/intake/RetractIntake.h"

RetractIntakeCommand::RetractIntakeCommand (Intake* intake) {
    AddRequirements(intake);
    mIntake = intake;
}

void RetractIntakeCommand::Initialize () {
    mIntake->retract();
}

void RetractIntakeCommand::Execute () {}

void RetractIntakeCommand::End (bool interrupted) {}

bool RetractIntakeCommand::IsFinished () {
    return true;
}