#include <commands/servoPower/PowerServosOn.h>

PowerServosOnCommand::PowerServosOnCommand(ServoPower * servoPower) {
    AddRequirements(servoPower);
    mServoPower = servoPower;
}

void PowerServosOnCommand::Initialize() {
    mServoPower->toggleServoPower(true);
}

void PowerServosOnCommand::Execute() {}

void PowerServosOnCommand::End(bool isInterrupted) {}

bool PowerServosOnCommand::IsFinished() {
    return true;
}