#include "commands/climber/Cycle.h"
#include <chrono>
#include <frc2/command/WaitCommand.h>

using namespace std::literals::chrono_literals;

const std::chrono::seconds kServoWaitTime = 1s;

Cycle::Cycle(Climber * climber, std::shared_ptr<cpptoml::table> toml) {
    AddRequirements(climber);
    mClimber = climber;

    config.armRotationForNextBar = toml->get_qualified_as<double>("armRotationForNextBar").value_or(0.0);
    config.armRotationVertical = toml->get_qualified_as<double>("armRotationVertical").value_or(0.5);
    config.dropToNextBar = toml->get_qualified_as<double>("dropToNextBar").value_or(0.0);
    config.extendToNextBarExtension = toml->get_qualified_as<double>("extendToNextBarExtension").value_or(0.0);
    config.extendToRearBar = toml->get_qualified_as<double>("extendToRearBar").value_or(0.0);
    config.grabNextBarExtension = toml->get_qualified_as<double>("grabNextBarExtension").value_or(0.0);
    config.initalExtension = toml->get_qualified_as<double>("initalExtension").value_or(0.0);
    config.liftExtension = toml->get_qualified_as<double>("liftExtension").value_or(0.0);
    config.dropOffRearBar = toml->get_qualified_as<double>("dropOffRearBar").value_or(0.0);
    config.releaseRearBar = toml->get_qualified_as<double>("releaseRearBar").value_or(0.0);
    config.restingExtension = toml->get_qualified_as<double>("restingExtension").value_or(0.0);

    mHighCycle = new frc2::SequentialCommandGroup(
        RotateOuterArmsCommand {mClimber, config.armRotationForNextBar},
        frc2::WaitCommand {kServoWaitTime},
        RetractInnerArmsCommand {mClimber, config.liftExtension},
        ExtendOuterArmsCommand {mClimber, config.extendToNextBarExtension},
        RotateOuterArmsCommand {mClimber, config.dropToNextBar},
        frc2::WaitCommand {kServoWaitTime},
        PowerServosOffCommand {mClimber},
        RetractOuterArmsCommand {mClimber, config.grabNextBarExtension},
        frc2::ParallelCommandGroup (
            ExtendInnerArmsCommand {mClimber, config.extendToRearBar}, // the inner arms extend
            RetractOuterArmsCommand {mClimber, config.liftExtension} // and the outer arms retract to bring it into position
        ),
        ExtendInnerArmsCommand {mClimber, config.releaseRearBar},
        RotateInnerArmsCommand {mClimber, config.dropOffRearBar},
        frc2::WaitCommand {kServoWaitTime},
        RetractInnerArmsCommand {mClimber, config.restingExtension}
    );

    mTraversalCycle = new frc2::SequentialCommandGroup (
        RotateInnerArmsCommand {mClimber, config.armRotationForNextBar},
        frc2::WaitCommand {kServoWaitTime},
        ExtendInnerArmsCommand {mClimber, config.extendToNextBarExtension},
        RotateInnerArmsCommand {mClimber, config.dropToNextBar},
        frc2::WaitCommand {kServoWaitTime},
        PowerServosOffCommand {mClimber},
        RetractInnerArmsCommand {mClimber, config.grabNextBarExtension},
        frc2::ParallelCommandGroup (
            ExtendOuterArmsCommand {mClimber, config.extendToRearBar}, // the outer arms extend
            RetractInnerArmsCommand {mClimber, config.liftExtension} // and the outer arms retract to bring it into position
        ),
        ExtendOuterArmsCommand {mClimber, config.releaseRearBar},
        RotateOuterArmsCommand {mClimber, config.dropOffRearBar},
        frc2::WaitCommand {kServoWaitTime},
        RetractOuterArmsCommand {mClimber, config.restingExtension},
        LockArmsCommand {mClimber}, // this command stops the arms from extending or retracting
        RotateOuterArmsCommand {mClimber, config.armRotationVertical}
    );
}

void Cycle::Initialize() {
    switch (mGoal) {
        case HIGH:
            mHighCycle->Schedule(false);
            mGoal = TRAVERSE;
            break;
    
        case TRAVERSE:
            mTraversalCycle->Schedule(false);
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