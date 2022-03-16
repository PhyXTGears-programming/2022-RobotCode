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

ClimbMidBarOnly::ClimbMidBarOnly(ClimberInnerReach * innerArms, ClimberInnerRotate * innerRotate, std::shared_ptr<cpptoml::table> toml) {
    config.initialExtension = toml->get_qualified_as<double>("initialExtension").value_or(config.initialExtension);
    config.liftRetraction = toml->get_qualified_as<double>("liftRetraction").value_or(config.liftRetraction);
    config.verticalArmAngle = toml->get_qualified_as<double>("verticalArmAngle").value_or(config.verticalArmAngle);

    mReachMidBar = new frc2::SequentialCommandGroup {
        frc2::InstantCommand {
            [=]() {
                //innerArms->unlockArms();
                innerArms->setUnderLoad(false);
            },
            { innerArms }
        },
        RotateInnerArmsCommand { innerRotate, config.verticalArmAngle },
        ExtendInnerArmsCommand { innerArms, config.initialExtension }
    };

    mClimbMidBar = new frc2::SequentialCommandGroup {
        frc2::InstantCommand {
            [=] () {
                innerArms->setUnderLoad(true);
            },
            { innerArms }
        },
        RetractInnerArmsCommand { innerArms, config.liftRetraction },
    };
}
