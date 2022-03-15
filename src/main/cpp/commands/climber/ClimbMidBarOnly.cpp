#include "commands/climber/ClimbMidBarOnly.h"

#include "commands/climber/ExtendInnerArms.h"
#include "commands/climber/RetractInnerArms.h"
#include "commands/climber/RotateInnerArms.h"

#include "commands/climber/ExtendOuterArms.h"
#include "commands/climber/RetractOuterArms.h"
#include "commands/climber/RotateOuterArms.h"

#include "commands/climber/LockArms.h"

#include <frc2/command/InstantCommand.h>

// DO NOT SCHEDULE THIS CLASS.  It is a container for related commands.  Use
// mReachMidBar and mClimbMidbarAndLock directly.

ClimbMidBarOnly::ClimbMidBarOnly(Climber * climber, std::shared_ptr<cpptoml::table> toml) {
    config.initialExtension = toml->get_qualified_as<double>("initialExtension").value_or(config.initialExtension);
    config.liftRetraction = toml->get_qualified_as<double>("liftRetraction").value_or(config.liftRetraction);
    config.verticalArmAngle = toml->get_qualified_as<double>("verticalArmAngle").value_or(config.verticalArmAngle);

    mReachMidBar = new frc2::SequentialCommandGroup(
        frc2::InstantCommand {
            [=]() {
                climber->setOuterUnderLoad(false);
            },
            { climber }
        },
        // RotateOuterArmsCommand { climber, config.verticalArmAngle },
        ExtendOuterArmsCommand { climber, config.initialExtension }
    );

    mClimbMidBarAndLock = new frc2::SequentialCommandGroup(
        frc2::InstantCommand {
            [=] () {
                climber->setOuterUnderLoad(true);
            },
            { climber }
        },
        RetractOuterArmsCommand { climber, config.liftRetraction }
    );
}