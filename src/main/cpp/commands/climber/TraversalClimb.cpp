#include "commands/climber/TraversalClimb.h"
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

TraversalClimb::TraversalClimb(Intake * intake, ClimberInnerReach * innerReach, ClimberInnerRotate * innerRotate, ClimberOuterReach * outerReach, ClimberOuterRotate * outerRotate, std::shared_ptr<cpptoml::table> toml) {
    AddRequirements(intake);
    AddRequirements(innerReach);
    AddRequirements(innerRotate);
    AddRequirements(outerReach);
    AddRequirements(outerRotate);
    
    config.inner.verticalArmAngle            = toml->get_qualified_as<double>("inner.verticalArmAngle").value_or(0.0);
    config.inner.zeroExtension               = toml->get_qualified_as<double>("inner.zeroExtension").value_or(0.0);
    config.inner.backOffAngle                = toml->get_qualified_as<double>("inner.backOffAngle").value_or(0.0);
    config.inner.dropOffPreviousBarAngle     = toml->get_qualified_as<double>("inner.dropOffPreviousBarAngle").value_or(0.0);
    config.inner.dropToNextBarAngle          = toml->get_qualified_as<double>("inner.dropToNextBarAngle").value_or(0.0);
    config.inner.grabNextBarExtension        = toml->get_qualified_as<double>("inner.grabNextBarExtension").value_or(0.0);
    config.inner.liftExtension               = toml->get_qualified_as<double>("inner.liftExtension").value_or(0.0);
    config.inner.liftOffExtension            = toml->get_qualified_as<double>("inner.liftOffExtension").value_or(0.0);
    config.inner.nextBarAngle                = toml->get_qualified_as<double>("inner.nextBarAngle").value_or(0.0);
    config.inner.nextBarExtension            = toml->get_qualified_as<double>("inner.nextBarExtension").value_or(0.0);
    config.inner.releasePreviousBarExtension = toml->get_qualified_as<double>("inner.releasePreviousBarExtension").value_or(0.0);
    config.inner.toPreviousBarExtension      = toml->get_qualified_as<double>("inner.toPreviousBarExtension").value_or(0.0);

    config.outer.verticalArmAngle            = toml->get_qualified_as<double>("outer.verticalArmAngle").value_or(0.0);
    config.outer.zeroExtension               = toml->get_qualified_as<double>("outer.zeroExtension").value_or(0.0);
    config.outer.backOffAngle                = toml->get_qualified_as<double>("outer.backOffAngle").value_or(0.0);
    config.outer.dropOffPreviousBarAngle     = toml->get_qualified_as<double>("outer.dropOffPreviousBarAngle").value_or(0.0);
    config.outer.dropToNextBarAngle          = toml->get_qualified_as<double>("outer.dropToNextBarAngle").value_or(0.0);
    config.outer.grabNextBarExtension        = toml->get_qualified_as<double>("outer.grabNextBarExtension").value_or(0.0);
    config.outer.liftExtension               = toml->get_qualified_as<double>("outer.liftExtension").value_or(0.0);
    config.outer.liftOffExtension            = toml->get_qualified_as<double>("outer.liftOffExtension").value_or(0.0);
    config.outer.nextBarAngle                = toml->get_qualified_as<double>("outer.nextBarAngle").value_or(0.0);
    config.outer.nextBarExtension            = toml->get_qualified_as<double>("outer.nextBarExtension").value_or(0.0);
    config.outer.releasePreviousBarExtension = toml->get_qualified_as<double>("outer.releasePreviousBarExtension").value_or(0.0);
    config.outer.toPreviousBarExtension      = toml->get_qualified_as<double>("outer.toPreviousBarExtension").value_or(0.0);

    mTraversalClimb = new frc2::SequentialCommandGroup {
        // Intake: move out of the way of the climb arms.
        ExtendIntakeCommand { intake },
        frc2::InstantCommand {[=]() { innerRotate->resetCurrentLimit(); }},
        frc2::InstantCommand {[=]() { outerRotate->resetCurrentLimit(); }},
        frc2::InstantCommand {[=]() { outerReach->setUnderLoad(true); }}.WithTimeout(1_s),
        // Outer: lift robot to mid bar.
        // Inner: point at high bar.
        // Prepare to grab high bar and swing.
        frc2::ParallelCommandGroup {
            RetractOuterArmsCommand {outerReach, config.outer.zeroExtension},
            frc2::ParallelRaceGroup {
                ExtendInnerArmsCommand {innerReach, config.inner.nextBarExtension},
                RotateInnerArmsCommand {innerRotate, config.inner.nextBarAngle},
            }
        },
        // Inner: grab high bar and swing.  Stop when robot under high bar.
        // Inner: leave robot low enough to release mid bar.
        // Prepare to release mid bar.
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
        // Outer: release mid bar and retract to clear mid bar.
        // Prepare to rotate outer to vertical.
        ExtendOuterArmsCommand {outerReach, config.outer.releasePreviousBarExtension},
        RotateOuterArmsCommand {outerRotate, config.outer.dropOffPreviousBarAngle}.WithTimeout(2_s),
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.outer.dropOffPreviousBarAngle},
            RetractOuterArmsCommand {outerReach, config.outer.zeroExtension},
        },
        frc2::InstantCommand {[=]() { innerRotate->resetCurrentLimit(); }},
        frc2::InstantCommand {[=]() { outerRotate->resetCurrentLimit(); }},
        // Outer: rotate toward next bar. Pass under high bar.
        RotateOuterArmsCommand {outerRotate, config.outer.nextBarAngle}.WithTimeout(2_s),
        // Outer: rotate to traverse bar and extend to traverse bar.
        // Inner: retract to raise robot and move outer closer to traverse bar.
        // Prepare to grab traverse bar.
        frc2::ParallelRaceGroup {
            RotateOuterArmsCommand {outerRotate, config.outer.nextBarAngle},
            frc2::SequentialCommandGroup {
                // Delay swing and give rotation time to clear high bar.
                frc2::WaitCommand {1_s},
                RetractInnerArmsCommand {innerReach, config.inner.zeroExtension},
                ExtendOuterArmsCommand {outerReach, config.outer.nextBarExtension}
            }
        },
        // Outer: grab traverse bar and swing.  Stop when robot under traverse bar.
        // Outer: leave robot low enough to release high bar.
        // Prepare to release high bar with inner arm.
        frc2::InstantCommand {[=]() { outerRotate->setCurrentlimit(15); }},
        RotateOuterArmsCommand {outerRotate, config.outer.dropToNextBarAngle}.WithTimeout(1_s),
        frc2::InstantCommand {[=](){ outerReach->setUnderLoad(true); innerReach->setUnderLoad(false); }},
        frc2::ParallelCommandGroup {
            RetractOuterArmsCommand {outerReach, config.outer.liftExtension},
            ExtendInnerArmsCommand {innerReach, config.inner.toPreviousBarExtension}
        },
        // Inner: release high bar and rotate hook below high bar, then retract
        // to clear high bar.
        // Prepare to rotate inner to vertical.
        ExtendInnerArmsCommand {innerReach, config.inner.releasePreviousBarExtension},
        frc2::ParallelRaceGroup {
            RotateInnerArmsCommand {innerRotate, config.inner.dropOffPreviousBarAngle},
            frc2::SequentialCommandGroup {
                frc2::WaitCommand {1_s},
                RetractInnerArmsCommand {innerReach, config.inner.liftExtension}
            }
        },
        // Inner: rotate near vertical.
        // Outer: lift robot against traverse bar, so inner hooks clear traverse bar.
        // Prepare to grab traverse bar with inner arm.
        frc2::ParallelCommandGroup {
            RotateInnerArmsCommand {innerRotate, config.inner.verticalArmAngle, 0.05, 0.075}.WithTimeout(2_s),
            frc2::SequentialCommandGroup {
                frc2::WaitCommand {1_s},
                RetractOuterArmsCommand {outerReach, config.outer.zeroExtension}
            }
        },
        // Inner: rotate to vertical.  gently crash traverse bar.
        frc2::InstantCommand {[=](){ innerRotate->setCurrentlimit(15); }},
        RotateInnerArmsCommand {innerRotate, config.inner.verticalArmAngle}.WithTimeout(1_s),
        // Inner: retract to grab traverse bar.
        // Outer: retract to keep robot against traverse bar.
        // Raise intake.
        frc2::ParallelRaceGroup {
            RotateInnerArmsCommand {innerRotate, config.inner.verticalArmAngle},
            frc2::ParallelCommandGroup {
                RetractInnerArmsCommand {innerReach, config.inner.zeroExtension},
                RetractOuterArmsCommand {outerReach, config.outer.zeroExtension}.AndThen([=](){  }),
            }
        },
        RetractIntakeCommand {intake}
    };
}

void TraversalClimb::Initialize() {
    mTraversalClimb->Initialize();
}

void TraversalClimb::Execute() {
    mTraversalClimb->Execute();
}

void TraversalClimb::End(bool interrupted) {
    mTraversalClimb->End(interrupted);
}

bool TraversalClimb::IsFinished() {
    return mTraversalClimb->IsFinished();
}