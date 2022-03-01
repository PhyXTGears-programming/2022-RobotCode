#include "commands/shooter/ShootCommand.h"

ShootCommand::ShootCommand (Shooter* shooter) {
    AddRequirements(shooter);
    mShooter = shooter;
}

void ShootCommand::Initialize () {}

void ShootCommand::Execute () {
    mShooter->runShooter(0.5);
}

void ShootCommand::End (bool interrupted) {
    mShooter->stopShooter();
}

bool ShootCommand::IsFinished () {
    return false;
}


