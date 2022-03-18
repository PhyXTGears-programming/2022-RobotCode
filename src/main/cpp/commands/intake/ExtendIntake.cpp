#include "commands/intake/ExtendIntake.h"

ExtendIntakeCommand::ExtendIntakeCommand(Intake *intake) {
    AddRequirements(intake);
    mIntake = intake;
}

void ExtendIntakeCommand::Initialize() {
    mIntake->extend();
}

void ExtendIntakeCommand::Execute() {}

void ExtendIntakeCommand::End(bool interrupted) {}

bool ExtendIntakeCommand::IsFinished() {
    return true;
}