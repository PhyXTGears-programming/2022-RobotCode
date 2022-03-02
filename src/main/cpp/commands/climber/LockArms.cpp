#include "commands/climber/LockArms.h"

LockArmsCommand::LockArmsCommand(Climber * climber) {
    AddRequirements(climber);
    mClimber = climber;
}

void LockArmsCommand::Initialize() {
    mClimber->lockArms();
}

void LockArmsCommand::Execute() {}

void LockArmsCommand::End(bool isInterrupted) {}

bool LockArmsCommand::IsFinished() {
    return true;
}