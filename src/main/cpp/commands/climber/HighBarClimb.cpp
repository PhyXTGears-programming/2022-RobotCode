#include "commands/climber/HighBarClimb.h"
#include "commands/intake/RunIntake.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

HighBarClimb::HighBarClimb(Intake * intake, ClimberInnerReach * innerReach, ClimberInnerRotate * innerRotate, ClimberOuterReach * outerReach, ClimberOuterRotate * outerRotate, std::shared_ptr<cpptoml::table> toml) {
    AddRequirements(intake);
    AddRequirements(innerReach);
    AddRequirements(innerRotate);
    AddRequirements(outerReach);
    AddRequirements(outerRotate);
    
    config.inner.verticalArmAngle = toml->get_qualified_as<double>("inner.verticalArmAngle").value_or(0.0);
    config.inner.zeroExtension = toml->get_qualified_as<double>("inner.zeroExtension").value_or(0.0);
    config.inner.backOffAngle = toml->get_qualified_as<double>("inner.backOffAngle").value_or(0.0);
    config.inner.dropOffPreviousBarAngle = toml->get_qualified_as<double>("inner.dropOffPreviousBarAngle").value_or(0.0);
    config.inner.dropToNextBarAngle = toml->get_qualified_as<double>("inner.dropToNextBarAngle").value_or(0.0);
    config.inner.grabNextBarExtension = toml->get_qualified_as<double>("inner.grabNextBarExtension").value_or(0.0);
    config.inner.liftExtension = toml->get_qualified_as<double>("inner.liftExtension").value_or(0.0);
    config.inner.liftOffExtension = toml->get_qualified_as<double>("inner.liftOffExtension").value_or(0.0);
    config.inner.nextBarAngle = toml->get_qualified_as<double>("inner.nextBarAngle").value_or(0.0);
    config.inner.nextBarExtension = toml->get_qualified_as<double>("inner.nextBarExtension").value_or(0.0);
    config.inner.releasePreviousBarExtension = toml->get_qualified_as<double>("inner.releasePreviousBarExtension").value_or(0.0);
    config.inner.toPreviousBarExtension = toml->get_qualified_as<double>("inner.toPreviousBarExtension").value_or(0.0);

    config.outer.verticalArmAngle = toml->get_qualified_as<double>("outer.verticalArmAngle").value_or(0.0);
    config.outer.zeroExtension = toml->get_qualified_as<double>("outer.zeroExtension").value_or(0.0);
    config.outer.backOffAngle = toml->get_qualified_as<double>("outer.backOffAngle").value_or(0.0);
    config.outer.dropOffPreviousBarAngle = toml->get_qualified_as<double>("outer.dropOffPreviousBarAngle").value_or(0.0);
    config.outer.dropToNextBarAngle = toml->get_qualified_as<double>("outer.dropToNextBarAngle").value_or(0.0);
    config.outer.grabNextBarExtension = toml->get_qualified_as<double>("outer.grabNextBarExtension").value_or(0.0);
    config.outer.liftExtension = toml->get_qualified_as<double>("outer.liftExtension").value_or(0.0);
    config.outer.liftOffExtension = toml->get_qualified_as<double>("outer.liftOffExtension").value_or(0.0);
    config.outer.nextBarAngle = toml->get_qualified_as<double>("outer.nextBarAngle").value_or(0.0);
    config.outer.nextBarExtension = toml->get_qualified_as<double>("outer.nextBarExtension").value_or(0.0);
    config.outer.releasePreviousBarExtension = toml->get_qualified_as<double>("outer.releasePreviousBarExtension").value_or(0.0);
    config.outer.toPreviousBarExtension = toml->get_qualified_as<double>("outer.toPreviousBarExtension").value_or(0.0);

    mHighBarClimb = new frc2::SequentialCommandGroup {
        RunIntakeCommand { intake }.WithTimeout(0.25_s),
        ExtendIntakeCommand { intake },
        frc2::InstantCommand {[=]() { innerRotate->resetCurrentLimit(); }},
        frc2::InstantCommand {[=]() { outerRotate->resetCurrentLimit(); }},
        frc2::InstantCommand {[=]() { outerReach->setUnderLoad(true); }}.WithTimeout(1_s),
        frc2::ParallelCommandGroup {
            RetractOuterArmsCommand {outerReach, config.outer.zeroExtension},
            frc2::ParallelRaceGroup {
                ExtendInnerArmsCommand {innerReach, config.inner.nextBarExtension},
                RotateInnerArmsCommand {innerRotate, config.inner.nextBarAngle},
            }
        },
        frc2::InstantCommand {[=]() { innerRotate->setCurrentlimit(15); }},
        RotateInnerArmsCommand {innerRotate, config.inner.dropToNextBarAngle}.WithTimeout(1_s),
        frc2::InstantCommand {[=]() { outerReach->setUnderLoad(false); }},
        frc2::InstantCommand {[=]() { innerReach->setUnderLoad(true); }},
        frc2::ParallelRaceGroup {
            RotateInnerArmsCommand {innerRotate, config.inner.dropToNextBarAngle},
            frc2::ParallelCommandGroup {
                RetractInnerArmsCommand {innerReach, config.inner.liftExtension},
                ExtendOuterArmsCommand {outerReach, config.outer.toPreviousBarExtension}
            }
        },
        ExtendOuterArmsCommand {outerReach, config.outer.releasePreviousBarExtension},
        RotateOuterArmsCommand {outerRotate, config.outer.dropOffPreviousBarAngle}.WithTimeout(1.5_s),
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.outer.dropOffPreviousBarAngle},
            RetractOuterArmsCommand {outerReach, config.outer.liftExtension},
        },
        frc2::ParallelCommandGroup {
            RotateOuterArmsCommand {outerRotate, config.outer.verticalArmAngle, 0.05, 0.075}.WithTimeout(2_s),
            frc2::SequentialCommandGroup {
                frc2::WaitCommand {1_s},
                RetractInnerArmsCommand {innerReach, config.inner.zeroExtension},
            }
        },
        frc2::InstantCommand {[=]() { outerRotate->setCurrentlimit(10); }},
        RotateOuterArmsCommand {outerRotate, config.outer.verticalArmAngle}.WithTimeout(1_s),
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.outer.verticalArmAngle},
            frc2::ParallelCommandGroup {
                RetractInnerArmsCommand {innerReach, config.inner.zeroExtension},
                RetractOuterArmsCommand {outerReach, config.outer.zeroExtension},
            }
        },
        RetractIntakeCommand {intake}
    };
}

void HighBarClimb::Initialize() {
    mHighBarClimb->Initialize();
}

void HighBarClimb::Execute() {
    mHighBarClimb->Execute();
}

void HighBarClimb::End(bool interrupted) {
    mHighBarClimb->End(interrupted);
}

bool HighBarClimb::IsFinished() {
    return mHighBarClimb->IsFinished();
}