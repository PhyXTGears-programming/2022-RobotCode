#include "commands/climber/Cycle.h"

Cycle::Cycle(Climber * climber, ServoPower * servoPower) {
    AddRequirements(climber);
    AddRequirements(servoPower);
    mClimber = climber;
    mServoPower = servoPower;
}

void Cycle::Initialize() {
    switch (mGoal) {
        case HIGH:
            highCycle->Schedule(false);
            mGoal = TRAVERSE;
            break;
    
        case TRAVERSE:
            traversalCycle->Schedule(false);
            mGoal = INOPERATIVE;
            break;
    }
}

void Cycle::Execute() {}

void Cycle::End(bool interrupted) {
    if (interrupted) {
        mGoal = INOPERATIVE;
    }
}

bool Cycle::IsFinished() {
    return true;
}