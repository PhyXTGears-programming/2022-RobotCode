#include "commands/climber/LockArms.h"

LockArmsCommand::LockArmsCommand(ClimberInnerReach * innerArms) {
    AddRequirements(innerArms);
    mInnerArms = innerArms;
}

void LockArmsCommand::Initialize() {
    //mInnerArms->lockArms();
}

void LockArmsCommand::Execute() {}

void LockArmsCommand::End(bool isInterrupted) {}

bool LockArmsCommand::IsFinished() {
    return true;
}