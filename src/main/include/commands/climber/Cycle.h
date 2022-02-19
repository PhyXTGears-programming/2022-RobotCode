#pragma once

#include "climber/climber.h"
#include "servoPower/servoPower.h"

#include "commands/climber/ExtendInnerArms.h"
#include "commands/climber/ExtendOuterArms.h"
#include "commands/climber/RetractOuterArms.h"
#include "commands/climber/RetractInnerArms.h"
#include "commands/climber/RotateOuterArms.h"
#include "commands/climber/RotateInnerArms.h"
#include "commands/climber/LockArms.h"
#include "commands/servoPower/PowerServosOff.h"
#include "commands/servoPower/PowerServosOn.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

class Cycle : public frc2::CommandHelper<frc2::CommandBase, Cycle> {
    public:
        Cycle(Climber * climber, ServoPower * servoPower);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        Climber * mClimber;
        ServoPower * mServoPower;

        enum CycleState {HIGH, TRAVERSE, INOPERATIVE = -1};
        CycleState mGoal = HIGH;
        
        struct {
            double initalExtension; // amount to raise arms to get on bar
            double liftExtension; // the amount to retract arms to lift robot up
            double armRotationForNextBar; // lifts arm above next rung
            double extendToNextBarExtension; // Reaches beyond the next rung
            double dropToNextBar; // drops arm onto next rung
            double grabNextBarExtension; // retracts arm so hook is grabbing next rung
            double releaseRearBar; // extends arm beyond rear bar
            double liftOffRearBar;
            double extendToRearBar; // extends the arms on the rear bar as the robot swings
            double armRotationVertical; // brings the arms vertical
            double restingExtension; // extension while driving/not reaching
        } config;

        frc2::SequentialCommandGroup * highCycle = new frc2::SequentialCommandGroup(
            RotateOuterArmsCommand {mClimber, config.armRotationForNextBar},
            RetractInnerArmsCommand {mClimber, config.liftExtension},
            ExtendOuterArmsCommand {mClimber, config.extendToNextBarExtension},
            RotateOuterArmsCommand {mClimber, config.dropToNextBar},
            RetractOuterArmsCommand {mClimber, config.grabNextBarExtension},
            PowerServosOffCommand {mServoPower},
            frc2::ParallelCommandGroup (
                ExtendInnerArmsCommand {mClimber, config.extendToRearBar}, // the inner arms extend
                RetractOuterArmsCommand {mClimber, config.liftExtension} // and the outer arms retract to bring it into position
            ),
            PowerServosOnCommand {mServoPower},
            ExtendInnerArmsCommand {mClimber, config.releaseRearBar},
            RotateInnerArmsCommand {mClimber, config.liftOffRearBar},
            RetractInnerArmsCommand {mClimber, config.restingExtension}
        );

        // at this point the outer hooks should be on the second bar and the inner hooks should be pointing towards the traversal rung.

        frc2::SequentialCommandGroup * traversalCycle = new frc2::SequentialCommandGroup (
            RotateInnerArmsCommand {mClimber, config.armRotationForNextBar},
            ExtendInnerArmsCommand {mClimber, config.extendToNextBarExtension},
            RotateInnerArmsCommand {mClimber, config.dropToNextBar},
            RetractInnerArmsCommand {mClimber, config.grabNextBarExtension},
            PowerServosOffCommand {mServoPower},
            frc2::ParallelCommandGroup (
                ExtendOuterArmsCommand {mClimber, config.extendToRearBar}, // the outer arms extend
                RetractInnerArmsCommand {mClimber, config.liftExtension} // and the outer arms retract to bring it into position
            ),
            PowerServosOnCommand {mServoPower},
            ExtendOuterArmsCommand {mClimber, config.releaseRearBar},
            RotateOuterArmsCommand {mClimber, config.liftOffRearBar},
            RetractOuterArmsCommand {mClimber, config.restingExtension},
            RotateOuterArmsCommand {mClimber, config.armRotationVertical},
            LockArmsCommand {mClimber}
        );
        // and now the inner hooks should be on the traversal rung.
};