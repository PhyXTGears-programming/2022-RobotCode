#include <commands/servoPower/PowerServosOff.h>

PowerServosOffCommand::PowerServosOffCommand(ServoPower * servoPower) {
    AddRequirements(servoPower);
    mServoPower = servoPower;
}

void PowerServosOffCommand::Initialize() {
    mServoPower->toggleServoPower(false);
}

void PowerServosOffCommand::Execute() {}

void PowerServosOffCommand::End(bool isInterrupted) {}

bool PowerServosOffCommand::IsFinished() {
    return true;
}