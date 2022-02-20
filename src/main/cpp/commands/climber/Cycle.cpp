#include "commands/climber/Cycle.h"

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
    config.liftOffRearBar = toml->get_qualified_as<double>("liftOffRearBar").value_or(0.0);
    config.releaseRearBar = toml->get_qualified_as<double>("releaseRearBar").value_or(0.0);
    config.restingExtension = toml->get_qualified_as<double>("restingExtension").value_or(0.0);

    mHighCycle = new frc2::SequentialCommandGroup(
        RotateOuterArmsCommand {mClimber, config.armRotationForNextBar},
        RetractInnerArmsCommand {mClimber, config.liftExtension},
        ExtendOuterArmsCommand {mClimber, config.extendToNextBarExtension},
        RotateOuterArmsCommand {mClimber, config.dropToNextBar},
        RetractOuterArmsCommand {mClimber, config.grabNextBarExtension},
        PowerServosOffCommand {mClimber},
        frc2::ParallelCommandGroup (
            ExtendInnerArmsCommand {mClimber, config.extendToRearBar}, // the inner arms extend
            RetractOuterArmsCommand {mClimber, config.liftExtension} // and the outer arms retract to bring it into position
        ),
        ExtendInnerArmsCommand {mClimber, config.releaseRearBar},
        RotateInnerArmsCommand {mClimber, config.liftOffRearBar},
        RetractInnerArmsCommand {mClimber, config.restingExtension}
    );

    mTraversalCycle = new frc2::SequentialCommandGroup (
        RotateInnerArmsCommand {mClimber, config.armRotationForNextBar},
        ExtendInnerArmsCommand {mClimber, config.extendToNextBarExtension},
        RotateInnerArmsCommand {mClimber, config.dropToNextBar},
        RetractInnerArmsCommand {mClimber, config.grabNextBarExtension},
        PowerServosOffCommand {mClimber},
        frc2::ParallelCommandGroup (
            ExtendOuterArmsCommand {mClimber, config.extendToRearBar}, // the outer arms extend
            RetractInnerArmsCommand {mClimber, config.liftExtension} // and the outer arms retract to bring it into position
        ),
        ExtendOuterArmsCommand {mClimber, config.releaseRearBar},
        RotateOuterArmsCommand {mClimber, config.liftOffRearBar},
        RetractOuterArmsCommand {mClimber, config.restingExtension},
        RotateOuterArmsCommand {mClimber, config.armRotationVertical},
        LockArmsCommand {mClimber}
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