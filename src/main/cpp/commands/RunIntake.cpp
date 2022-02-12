#include "commands/RunIntake.h"

RunIntakeCommand::RunIntakeCommand (Intake* intake) {
    AddRequirements(intake);
    mIntake = intake;
}

void RunIntakeCommand::Initialize () {
    mIntake->runRollers();
}

void RunIntakeCommand::Execute () {}

void RunIntakeCommand::End (bool interrupted) {
    mIntake->stopRollers();
}

bool RunIntakeCommand::IsFinished () {
    return false;
}