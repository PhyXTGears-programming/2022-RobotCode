#include "commands/climber/Cycle.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>

Cycle::Cycle(ClimberInnerReach * innerReach, ClimberInnerRotate * innerRotate, ClimberOuterReach * outerReach, ClimberOuterRotate * outerRotate, std::shared_ptr<cpptoml::table> toml) {
    AddRequirements(innerReach);
    AddRequirements(innerRotate);
    AddRequirements(outerReach);
    AddRequirements(outerRotate);
    
    config.cycle.verticalArmAngle = toml->get_qualified_as<double>("cycle.verticalArmAngle").value_or(0.0);
    config.cycle.zeroExtension = toml->get_qualified_as<double>("cycle.zeroExtension").value_or(0.0);
    config.cycle.backOffAngle = toml->get_qualified_as<double>("cycle.backOffAngle").value_or(0.0);
    config.cycle.dropOffPreviousBarAngle = toml->get_qualified_as<double>("cycle.dropOffPreviousBarAngle").value_or(0.0);
    config.cycle.dropToNextBarAngle = toml->get_qualified_as<double>("cycle.dropToNextBarAngle").value_or(0.0);
    config.cycle.grabNextBarExtension = toml->get_qualified_as<double>("cycle.grabNextBarExtension").value_or(0.0);
    config.cycle.liftExtension = toml->get_qualified_as<double>("cycle.liftExtension").value_or(0.0);
    config.cycle.liftOffExtension = toml->get_qualified_as<double>("cycle.liftOffExtension").value_or(0.0);
    config.cycle.nextBarAngle = toml->get_qualified_as<double>("cycle.nextBarAngle").value_or(0.0);
    config.cycle.nextBarExtension = toml->get_qualified_as<double>("cycle.nextBarExtension").value_or(0.0);
    config.cycle.releasePreviousBarExtension = toml->get_qualified_as<double>("cycle.releasePreviousBarExtension").value_or(0.0);
    config.cycle.toPreviousBarExtension = toml->get_qualified_as<double>("cycle.toPreviousBarExtension").value_or(0.0);

    // config.armAngleForNextBar = toml->get_qualified_as<double>("armAngleForNextBar").value_or(0.0);
    // config.armAngleVertical = toml->get_qualified_as<double>("armAngleVertical").value_or(0.5);
    // config.dropToNextBar = toml->get_qualified_as<double>("dropToNextBar").value_or(0.0);
    // config.extendToNextBarExtension = toml->get_qualified_as<double>("extendToNextBarExtension").value_or(0.0);
    // config.extendToRearBar = toml->get_qualified_as<double>("extendToRearBar").value_or(0.0);
    // config.grabNextBarExtension = toml->get_qualified_as<double>("grabNextBarExtension").value_or(0.0);
    // config.initalExtension = toml->get_qualified_as<double>("initalExtension").value_or(0.0);
    // config.liftExtension = toml->get_qualified_as<double>("liftExtension").value_or(2.42);
    // config.dropOffRearBar = toml->get_qualified_as<double>("dropOffRearBar").value_or(0.0);
    // config.releaseRearBar = toml->get_qualified_as<double>("releaseRearBar").value_or(0.0);
    // config.restingExtension = toml->get_qualified_as<double>("restingExtension").value_or(0.0);

    // mHighCycle = new frc2::SequentialCommandGroup(
    //     frc2::InstantCommand {[&]() { innerReach->setUnderLoad(true); }},
    //     frc2::ParallelCommandGroup {
    //       RetractInnerArmsCommand {innerReach, config.liftExtension},
    //       RotateOuterArmsCommand {outerRotate, config.armAngleForNextBar}
    //     },
    //     ExtendOuterArmsCommand {outerReach, config.extendToNextBarExtension},
    //     RotateOuterArmsCommand {outerRotate, config.dropToNextBar},
    //     RetractOuterArmsCommand {outerReach, config.grabNextBarExtension},
    //     frc2::InstantCommand {[&]() { outerRotate->setMotorCoast(); }},
    //     frc2::InstantCommand {[&]() { innerRotate->setMotorCoast(); }},
    //     frc2::InstantCommand {[&]() { outerReach->setUnderLoad(true); }},
    //     frc2::ParallelCommandGroup (
    //         ExtendInnerArmsCommand {innerReach, config.extendToRearBar}, // the inner arms extend
    //         RetractOuterArmsCommand {outerReach, config.liftExtension} // and the outer arms retract to bring it into position
    //     ),
    //     frc2::InstantCommand {[&]() { innerReach->setUnderLoad(false); }},
    //     ExtendInnerArmsCommand {innerReach, config.releaseRearBar},
    //     RotateInnerArmsCommand {innerRotate, config.dropOffRearBar},
    //     RetractInnerArmsCommand {innerReach, config.restingExtension}
    // );

    // mTraversalCycle = new frc2::SequentialCommandGroup (
    //     RotateInnerArmsCommand {innerRotate, config.armAngleForNextBar},
    //     ExtendInnerArmsCommand {innerReach, config.extendToNextBarExtension},
    //     RotateInnerArmsCommand {innerRotate, config.dropToNextBar},
    //     RetractInnerArmsCommand {innerReach, config.grabNextBarExtension},
    //     frc2::InstantCommand {[&]() { outerRotate->setMotorCoast(); }},
    //     frc2::InstantCommand {[&]() { innerRotate->setMotorCoast(); }},
    //     frc2::InstantCommand {[&]() { innerReach->setUnderLoad(true); }},
    //     frc2::ParallelCommandGroup (
    //         ExtendOuterArmsCommand {outerReach, config.extendToRearBar}, // the outer arms extend
    //         RetractInnerArmsCommand {innerReach, config.liftExtension} // and the inner arms retract to bring it into position
    //     ),
    //     frc2::InstantCommand {[&]() { outerReach->setUnderLoad(false); }},
    //     ExtendOuterArmsCommand {outerReach, config.releaseRearBar},
    //     RotateOuterArmsCommand {outerRotate, config.dropOffRearBar},
    //     RetractOuterArmsCommand {outerReach, config.liftExtension}, // pulls arms in so they don't hit bar on next step
    //     frc2::ParallelCommandGroup { // extend and rotate to vertical to grab traversal with both sets of arms
    //         //LockArmsCommand {innerReach}, // this command stops the arms from extending or retracting
    //         ExtendOuterArmsCommand {outerReach, config.initalExtension},
    //         RotateOuterArmsCommand {outerRotate, config.armAngleVertical}
    //     },
    //     frc2::InstantCommand {[&]() { outerReach->setUnderLoad(true); }},
    //     RetractOuterArmsCommand {outerReach, config.liftExtension} // tighten grip
    // );

    mCycle = new frc2::SequentialCommandGroup {
        ExtendInnerArmsCommand {innerReach, config.cycle.liftOffExtension},
        frc2::ParallelCommandGroup {
            RotateInnerArmsCommand {innerRotate, config.cycle.backOffAngle},
            RetractInnerArmsCommand {innerReach, config.cycle.zeroExtension}
        },
        frc2::ParallelCommandGroup {
            RotateInnerArmsCommand {innerRotate, config.cycle.nextBarAngle},
            ExtendInnerArmsCommand {innerReach, config.cycle.nextBarExtension}
        },
        frc2::ParallelCommandGroup {
            RotateInnerArmsCommand {innerRotate, config.cycle.dropToNextBarAngle},
            RetractInnerArmsCommand {innerReach, config.cycle.grabNextBarExtension}
        },
        frc2::ParallelCommandGroup {
            ExtendOuterArmsCommand {outerReach, config.cycle.toPreviousBarExtension},
            RetractInnerArmsCommand {innerReach, config.cycle.liftExtension}
        },
        frc2::ParallelCommandGroup {
            RotateOuterArmsCommand {outerRotate, config.cycle.dropOffPreviousBarAngle},
            ExtendOuterArmsCommand {outerReach, config.cycle.releasePreviousBarExtension}  
        },
        frc2::ParallelCommandGroup {
            RotateOuterArmsCommand {outerRotate, config.cycle.backOffAngle},
            RetractOuterArmsCommand {outerReach, config.cycle.zeroExtension}
        },
        frc2::ParallelCommandGroup {
            RotateOuterArmsCommand {outerRotate, config.cycle.verticalArmAngle},
            ExtendOuterArmsCommand {outerReach, config.cycle.liftOffExtension}
        },
        RetractOuterArmsCommand {outerReach, config.cycle.liftExtension}
    };
}

void Cycle::Initialize() {
    switch (mGoal) {
        case HIGH:
            mCycle->Schedule(false);
            mGoal = TRAVERSE;
            break;
    
        case TRAVERSE:
            mCycle->Schedule(false);
            mGoal = INOPERATIVE;
            break;
        
        case INOPERATIVE:
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