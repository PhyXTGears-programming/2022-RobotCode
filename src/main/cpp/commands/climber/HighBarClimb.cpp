#include "commands/climber/HighBarClimb.h"
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

HighBarClimb::HighBarClimb(Intake * intake, ClimberInnerReach * innerReach, ClimberInnerRotate * innerRotate, ClimberOuterReach * outerReach, ClimberOuterRotate * outerRotate, std::shared_ptr<cpptoml::table> toml) {
    AddRequirements(intake);
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

    mHighBarClimb = new frc2::SequentialCommandGroup {
        ExtendIntakeCommand { intake },
        frc2::InstantCommand {[=]() { innerRotate->resetCurrentLimit(); }},
        frc2::InstantCommand {[=]() { outerRotate->resetCurrentLimit(); }},
        frc2::InstantCommand {[=]() { outerReach->setUnderLoad(true); }}.WithTimeout(1_s),
        frc2::ParallelCommandGroup {
            RetractOuterArmsCommand {outerReach, config.cycle.outer.zeroExtension},
            frc2::ParallelRaceGroup {
                ExtendInnerArmsCommand {innerReach, config.cycle.inner.nextBarExtension},
                RotateInnerArmsCommand {innerRotate, config.cycle.inner.nextBarAngle},
            }
        },
        frc2::InstantCommand {[=]() { innerRotate->setCurrentlimit(15); }},
        RotateInnerArmsCommand {innerRotate, config.cycle.inner.dropToNextBarAngle}.WithTimeout(1_s),
        frc2::InstantCommand {[=]() { outerReach->setUnderLoad(false); }},
        frc2::InstantCommand {[=]() { innerReach->setUnderLoad(true); }},
        frc2::ParallelRaceGroup {
            RotateInnerArmsCommand {innerRotate, config.cycle.inner.dropToNextBarAngle},
            frc2::ParallelCommandGroup {
                RetractInnerArmsCommand {innerReach, config.cycle.inner.liftExtension},
                ExtendOuterArmsCommand {outerReach, config.cycle.outer.toPreviousBarExtension}
            }
        },
        ExtendOuterArmsCommand {outerReach, config.cycle.outer.releasePreviousBarExtension},
        RotateOuterArmsCommand {outerRotate, config.cycle.outer.dropOffPreviousBarAngle}.WithTimeout(2_s),
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.cycle.outer.dropOffPreviousBarAngle},
            RetractOuterArmsCommand {outerReach, config.cycle.outer.liftExtension},
        },
        frc2::ParallelCommandGroup {
            RotateOuterArmsCommand {outerRotate, config.cycle.outer.verticalArmAngle, 0.05, 0.075}.WithTimeout(2_s),
            frc2::SequentialCommandGroup {
                frc2::WaitCommand {1_s},
                RetractInnerArmsCommand {innerReach, config.cycle.inner.zeroExtension},
            }
        },
        frc2::InstantCommand {[=]() { outerRotate->setCurrentlimit(10); }},
        RotateOuterArmsCommand {outerRotate, config.cycle.outer.verticalArmAngle}.WithTimeout(1_s),
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.cycle.outer.verticalArmAngle},
            frc2::ParallelCommandGroup {
                RetractInnerArmsCommand {innerReach, config.cycle.inner.zeroExtension},
                RetractOuterArmsCommand {outerReach, config.cycle.outer.zeroExtension},
            }
        }

    };
}

void HighBarClimb::Initialize() {
    mHighBarClimb->Schedule();
}

void HighBarClimb::Execute() {}

void HighBarClimb::End(bool interrupted) {
    if (interrupted) {
        mGoal = INOPERATIVE;
    }
}

bool HighBarClimb::IsFinished() {
    return true;
}