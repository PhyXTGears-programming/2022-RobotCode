#include "commands/climber/Cycle.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>

Cycle::Cycle(Climber * climber, std::shared_ptr<cpptoml::table> toml) {
    AddRequirements(climber);
    mClimber = climber;

    config.armAngleForNextBar = toml->get_qualified_as<double>("armAngleForNextBar").value_or(0.0);
    config.armAngleVertical = toml->get_qualified_as<double>("armAngleVertical").value_or(0.5);
    config.dropToNextBar = toml->get_qualified_as<double>("dropToNextBar").value_or(0.0);
    config.extendToNextBarExtension = toml->get_qualified_as<double>("extendToNextBarExtension").value_or(0.0);
    config.extendToRearBar = toml->get_qualified_as<double>("extendToRearBar").value_or(0.0);
    config.grabNextBarExtension = toml->get_qualified_as<double>("grabNextBarExtension").value_or(0.0);
    config.initalExtension = toml->get_qualified_as<double>("initalExtension").value_or(0.0);
    config.liftExtension = toml->get_qualified_as<double>("liftExtension").value_or(2.42);
    config.dropOffRearBar = toml->get_qualified_as<double>("dropOffRearBar").value_or(0.0);
    config.releaseRearBar = toml->get_qualified_as<double>("releaseRearBar").value_or(0.0);
    config.restingExtension = toml->get_qualified_as<double>("restingExtension").value_or(0.0);

    mHighCycle = new frc2::SequentialCommandGroup(
        RotateOuterArmsCommand {mClimber, config.armAngleForNextBar},
        frc2::InstantCommand {[&]() { mClimber->setInnerUnderLoad(true); }},
        RetractInnerArmsCommand {mClimber, config.liftExtension},
        ExtendOuterArmsCommand {mClimber, config.extendToNextBarExtension},
        RotateOuterArmsCommand {mClimber, config.dropToNextBar},
        RetractOuterArmsCommand {mClimber, config.grabNextBarExtension},
        frc2::InstantCommand {[&]() { mClimber->setRotateMotorsCoast(); }},
        frc2::InstantCommand {[&]() { mClimber->setOuterUnderLoad(true); }},
        frc2::ParallelCommandGroup (
            ExtendInnerArmsCommand {mClimber, config.extendToRearBar}, // the inner arms extend
            RetractOuterArmsCommand {mClimber, config.liftExtension} // and the outer arms retract to bring it into position
        ),
        frc2::InstantCommand {[&]() { mClimber->setInnerUnderLoad(false); }},
        ExtendInnerArmsCommand {mClimber, config.releaseRearBar},
        RotateInnerArmsCommand {mClimber, config.dropOffRearBar},
        RetractInnerArmsCommand {mClimber, config.restingExtension}
    );

    mTraversalCycle = new frc2::SequentialCommandGroup (
        RotateInnerArmsCommand {mClimber, config.armAngleForNextBar},
        ExtendInnerArmsCommand {mClimber, config.extendToNextBarExtension},
        RotateInnerArmsCommand {mClimber, config.dropToNextBar},
        RetractInnerArmsCommand {mClimber, config.grabNextBarExtension},
        frc2::InstantCommand {[&]() { mClimber->setRotateMotorsCoast(); }},
        frc2::InstantCommand {[&]() { mClimber->setInnerUnderLoad(true); }},
        frc2::ParallelCommandGroup (
            ExtendOuterArmsCommand {mClimber, config.extendToRearBar}, // the outer arms extend
            RetractInnerArmsCommand {mClimber, config.liftExtension} // and the inner arms retract to bring it into position
        ),
        frc2::InstantCommand {[&]() { mClimber->setOuterUnderLoad(false); }},
        ExtendOuterArmsCommand {mClimber, config.releaseRearBar},
        RotateOuterArmsCommand {mClimber, config.dropOffRearBar},
        RetractOuterArmsCommand {mClimber, config.restingExtension},
        LockArmsCommand {mClimber}, // this command stops the arms from extending or retracting
        RotateOuterArmsCommand {mClimber, config.armAngleVertical}
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