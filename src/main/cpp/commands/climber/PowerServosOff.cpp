#include <commands/climber/PowerServosOff.h>

PowerServosOffCommand::PowerServosOffCommand(Climber * climber) {
    AddRequirements(climber);
    mClimber = climber;
}

void PowerServosOffCommand::Initialize() {
    mClimber->disableServos();
}

void PowerServosOffCommand::Execute() {}

void PowerServosOffCommand::End(bool isInterrupted) {}

bool PowerServosOffCommand::IsFinished() {
    return true;
}