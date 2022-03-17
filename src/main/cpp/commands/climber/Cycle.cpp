#include "commands/climber/Cycle.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>

Cycle::Cycle(ClimberInnerReach * innerReach, ClimberInnerRotate * innerRotate, ClimberOuterReach * outerReach, ClimberOuterRotate * outerRotate, std::shared_ptr<cpptoml::table> toml) {
    AddRequirements(innerReach);
    AddRequirements(innerRotate);
    AddRequirements(outerReach);
    AddRequirements(outerRotate);
    
    config.cycle.inner.verticalArmAngle = toml->get_qualified_as<double>("cycle.inner.verticalArmAngle").value_or(0.0);
    config.cycle.inner.zeroExtension = toml->get_qualified_as<double>("cycle.inner.zeroExtension").value_or(0.0);
    config.cycle.inner.backOffAngle = toml->get_qualified_as<double>("cycle.inner.backOffAngle").value_or(0.0);
    config.cycle.inner.dropOffPreviousBarAngle = toml->get_qualified_as<double>("cycle.inner.dropOffPreviousBarAngle").value_or(0.0);
    config.cycle.inner.dropToNextBarAngle = toml->get_qualified_as<double>("cycle.inner.dropToNextBarAngle").value_or(0.0);
    config.cycle.inner.grabNextBarExtension = toml->get_qualified_as<double>("cycle.inner.grabNextBarExtension").value_or(0.0);
    config.cycle.inner.liftExtension = toml->get_qualified_as<double>("cycle.inner.liftExtension").value_or(0.0);
    config.cycle.inner.liftOffExtension = toml->get_qualified_as<double>("cycle.inner.liftOffExtension").value_or(0.0);
    config.cycle.inner.nextBarAngle = toml->get_qualified_as<double>("cycle.inner.nextBarAngle").value_or(0.0);
    config.cycle.inner.nextBarExtension = toml->get_qualified_as<double>("cycle.inner.nextBarExtension").value_or(0.0);
    config.cycle.inner.releasePreviousBarExtension = toml->get_qualified_as<double>("cycle.inner.releasePreviousBarExtension").value_or(0.0);
    config.cycle.inner.toPreviousBarExtension = toml->get_qualified_as<double>("cycle.inner.toPreviousBarExtension").value_or(0.0);

    config.cycle.outer.verticalArmAngle = toml->get_qualified_as<double>("cycle.outer.verticalArmAngle").value_or(0.0);
    config.cycle.outer.zeroExtension = toml->get_qualified_as<double>("cycle.outer.zeroExtension").value_or(0.0);
    config.cycle.outer.backOffAngle = toml->get_qualified_as<double>("cycle.outer.backOffAngle").value_or(0.0);
    config.cycle.outer.dropOffPreviousBarAngle = toml->get_qualified_as<double>("cycle.outer.dropOffPreviousBarAngle").value_or(0.0);
    config.cycle.outer.dropToNextBarAngle = toml->get_qualified_as<double>("cycle.outer.dropToNextBarAngle").value_or(0.0);
    config.cycle.outer.grabNextBarExtension = toml->get_qualified_as<double>("cycle.outer.grabNextBarExtension").value_or(0.0);
    config.cycle.outer.liftExtension = toml->get_qualified_as<double>("cycle.outer.liftExtension").value_or(0.0);
    config.cycle.outer.liftOffExtension = toml->get_qualified_as<double>("cycle.outer.liftOffExtension").value_or(0.0);
    config.cycle.outer.nextBarAngle = toml->get_qualified_as<double>("cycle.outer.nextBarAngle").value_or(0.0);
    config.cycle.outer.nextBarExtension = toml->get_qualified_as<double>("cycle.outer.nextBarExtension").value_or(0.0);
    config.cycle.outer.releasePreviousBarExtension = toml->get_qualified_as<double>("cycle.outer.releasePreviousBarExtension").value_or(0.0);
    config.cycle.outer.toPreviousBarExtension = toml->get_qualified_as<double>("cycle.outer.toPreviousBarExtension").value_or(0.0);

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
            frc2::SequentialCommandGroup {
                frc2::WaitCommand {1_s},
                RetractInnerArmsCommand {innerReach, config.cycle.zeroExtension}
            }
        },
        RotateInnerArmsCommand {innerRotate, config.cycle.verticalArmAngle},
        frc2::ParallelCommandGroup {
            RotateInnerArmsCommand {innerRotate, config.cycle.nextBarAngle},
            ExtendInnerArmsCommand {innerReach, config.cycle.nextBarExtension}
        },
        frc2::ParallelCommandGroup {
            RotateInnerArmsCommand {innerRotate, config.cycle.dropToNextBarAngle},
            frc2::SequentialCommandGroup {
                frc2::WaitCommand {0.5_s},
                RetractInnerArmsCommand {innerReach, config.cycle.grabNextBarExtension}
            }
        },
        frc2::InstantCommand {[&]() { outerRotate->setMotorCoast(); innerRotate->setMotorCoast(); }},
        frc2::ParallelCommandGroup {
            ExtendOuterArmsCommand {outerReach, config.cycle.toPreviousBarExtension},
            RetractInnerArmsCommand {innerReach, config.cycle.liftExtension}
        },
        frc2::InstantCommand {[&]() { outerRotate->setMotorBrake(); innerRotate->setMotorBrake(); }}, // just in case.
        frc2::ParallelCommandGroup {
            ExtendOuterArmsCommand {outerReach, config.cycle.releasePreviousBarExtension},
            frc2::SequentialCommandGroup {
                frc2::WaitCommand {0.5_s},
                RotateOuterArmsCommand {outerRotate, config.cycle.dropOffPreviousBarAngle}
            }
        },
        RetractOuterArmsCommand {outerReach, config.cycle.zeroExtension},
        frc2::ParallelCommandGroup {
            RotateOuterArmsCommand {outerRotate, config.cycle.backOffAngle},
            ExtendOuterArmsCommand {outerReach, config.cycle.liftOffExtension}
        },
        RotateOuterArmsCommand {outerRotate, config.cycle.verticalArmAngle},
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