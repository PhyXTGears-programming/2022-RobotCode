#include "commands/climber/HighBarClimb.h"
#include "commands/climber/ReachInnerArms.h"
#include "commands/climber/ReachOuterArms.h"
#include "commands/intake/RunIntake.h"

#include "PID.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PerpetualCommand.h>
#include <frc2/command/PrintCommand.h>
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
    config.inner.insidePreviousBarExtension = toml->get_qualified_as<double>("inner.insidePreviousBarExtension").value_or(0.0);
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
    config.outer.insidePreviousBarExtension = toml->get_qualified_as<double>("outer.insidePreviousBarExtension").value_or(0.0);
    config.outer.nextBarAngle = toml->get_qualified_as<double>("outer.nextBarAngle").value_or(0.0);
    config.outer.nextBarExtension = toml->get_qualified_as<double>("outer.nextBarExtension").value_or(0.0);
    config.outer.releasePreviousBarExtension = toml->get_qualified_as<double>("outer.releasePreviousBarExtension").value_or(0.0);
    config.outer.toPreviousBarExtension = toml->get_qualified_as<double>("outer.toPreviousBarExtension").value_or(0.0);

    PID outerRotatePid { 0.02, 0.0, 0.0, 0.12, 0.01, -0.2, 0.2 };
    PID innerRotatePid { 0.02, 0.0, 0.0, 0.07, 0.01, -0.15, 0.15 };

    PID climbOuterPid { 0.3, 0.004, 0.0, 0.2, 0.05, -0.8, 0.05, 1.0 };
    PID climbInnerPid { 0.3, 0.004, 0.0, 0.2, 0.05, -0.8, 0.05, 1.0 };

    PID slowInnerPid { 0.3, 0.004, 0.0, 0.3, 0.05, -0.45, 0.20, 1.0 };
    PID slowOuterPid { 0.3, 0.004, 0.0, 0.3, 0.05, -0.45, 0.20, 1.0 };

    PID fastInnerReach { 0.4, 0.0, 0.0, 0.03, 0.05, -0.3, 0.3, 0.5 };

    mHighBarClimb = new frc2::SequentialCommandGroup {
        frc2::InstantCommand {[=]() { innerRotate->setCurrentlimit(15); }, {innerRotate}},
        frc2::InstantCommand {[=]() { outerRotate->setCurrentlimit(15); }, {outerRotate}},

        frc2::PrintCommand { "Begin high bar climb: drop intake" },
        RunIntakeCommand { intake }.WithTimeout(0.25_s),
        ExtendIntakeCommand { intake },

        frc2::PrintCommand { "Rotate inner arms toward high bar." },
        ReachInnerArmsCommand {innerReach, config.inner.zeroExtension, fastInnerReach, fastInnerReach},
        RotateInnerArmsCommand {innerRotate, config.inner.nextBarAngle, innerRotatePid},

        frc2::PrintCommand { "Lift robot onto mid bar.  Reach for high bar." },
        frc2::ParallelCommandGroup {
            ReachOuterArmsCommand {outerReach, config.outer.zeroExtension, climbOuterPid, climbOuterPid},
            frc2::ParallelRaceGroup {
                ReachInnerArmsCommand {innerReach, config.inner.nextBarExtension},
                RotateInnerArmsCommand {innerRotate, config.inner.nextBarAngle, innerRotatePid}.Perpetually(),
            }
        },

        frc2::PrintCommand { "Overdrive into high bar" },
        RotateInnerArmsCommand {innerRotate, config.inner.dropToNextBarAngle, innerRotatePid}.WithTimeout(1.0_s),

        frc2::PrintCommand { "Swing under high bar" },
        frc2::InstantCommand {[=]() { outerReach->setUnderLoad(false); innerReach->setUnderLoad(true); }, {innerReach, outerReach}},
        frc2::InstantCommand {[=]() { innerRotate->setMotorCoast(); outerRotate->setMotorCoast(); }, {innerRotate, outerRotate}},
        frc2::ParallelCommandGroup {
            ReachInnerArmsCommand {innerReach, config.inner.liftExtension, slowInnerPid, slowInnerPid},
            ReachOuterArmsCommand {outerReach, config.outer.toPreviousBarExtension, slowOuterPid, slowOuterPid}
        },

        frc2::PrintCommand { "Release mid bar" },
        frc2::InstantCommand {[=]() { innerRotate->setMotorBrake(); outerRotate->setMotorBrake(); }, {innerRotate, outerRotate}},
        ReachOuterArmsCommand {outerReach, config.outer.releasePreviousBarExtension, slowOuterPid, slowOuterPid},

        frc2::PrintCommand { "Rotate hook below mid bar" },
        RotateOuterArmsCommand {outerRotate, config.outer.dropOffPreviousBarAngle, outerRotatePid},

        frc2::PrintCommand { "Retract outer arm" },
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.outer.dropOffPreviousBarAngle, outerRotatePid}.Perpetually(),
            ReachOuterArmsCommand {outerReach, config.outer.insidePreviousBarExtension},
        },

        frc2::PrintCommand { "Lift robot.  Rotate outer to vertical" },
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.outer.verticalArmAngle, outerRotatePid}.Perpetually(),
            ReachInnerArmsCommand {innerReach, config.inner.zeroExtension, climbInnerPid, climbInnerPid},
        },

        frc2::PrintCommand { "Grab high bar and lift robot" },
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.outer.verticalArmAngle, outerRotatePid}.Perpetually(),
            frc2::ParallelCommandGroup {
                ReachInnerArmsCommand {innerReach, config.inner.zeroExtension, climbInnerPid, climbInnerPid},
                ReachOuterArmsCommand {outerReach, config.outer.zeroExtension, climbOuterPid, climbOuterPid},
            }
        },

        frc2::PrintCommand { "Retract intake... if it's still there" },
        RetractIntakeCommand {intake},

        frc2::InstantCommand {
            [=]() { innerRotate->setMotorBrake(); outerRotate->setMotorBrake(); },
            {innerRotate, outerRotate}
        }
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
