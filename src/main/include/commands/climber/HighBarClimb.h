#pragma once

#include "cpptoml.h"

#include "climber/InnerReach.h"
#include "climber/InnerRotate.h"
#include "climber/OuterReach.h"
#include "climber/OuterRotate.h"

#include "intake/intake.h"

#include "commands/climber/ExtendInnerArms.h"
#include "commands/climber/ExtendOuterArms.h"
#include "commands/climber/RetractOuterArms.h"
#include "commands/climber/RetractInnerArms.h"
#include "commands/climber/RotateOuterArms.h"
#include "commands/climber/RotateInnerArms.h"
#include "commands/climber/LockArms.h"

#include "commands/intake/ExtendIntake.h"
#include "commands/intake/RetractIntake.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

class HighBarClimb : public frc2::CommandHelper<frc2::CommandBase, HighBarClimb> {
    public:
        HighBarClimb(Intake * intake, ClimberInnerReach * innerReach, ClimberInnerRotate * innerRotate, ClimberOuterReach * outerReach, ClimberOuterRotate * outerRotate, std::shared_ptr<cpptoml::table> toml);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:

        Intake * mIntake = nullptr;

        ClimberInnerReach * mInnerReach = nullptr;
        ClimberOuterReach * mOuterReach = nullptr;
        ClimberInnerRotate * mInnerRotate = nullptr;
        ClimberOuterRotate * mOuterRotate = nullptr;

        enum CycleState {HIGH, TRAVERSE, INOPERATIVE = -1};
        CycleState mGoal = HIGH;
        
        struct {
            // double initalExtension; // amount to raise arms to get on bar
            // double liftExtension; // the amount to retract arms to lift robot up
            // double armAngleForNextBar; // lifts arm above next rung
            // double extendToNextBarExtension; // Reaches beyond the next rung
            // double dropToNextBar; // drops arm onto next rung
            // double grabNextBarExtension; // retracts arm so hook is grabbing next rung
            // double releaseRearBar; // extends arm beyond rear bar
            // double dropOffRearBar;
            // double extendToRearBar; // extends the arms on the rear bar as the robot swings
            // double armAngleVertical; // brings the arms vertical
            // double restingExtension; // extension while driving/not reaching

            struct {
                double liftOffExtension;
                double backOffAngle;
                double nextBarAngle;
                double nextBarExtension;
                double dropToNextBarAngle;
                double grabNextBarExtension;
                double verticalArmAngle;
                double liftExtension;
                double insidePreviousBarExtension;
                double toPreviousBarExtension;
                double zeroExtension;
                double releasePreviousBarExtension;
                double dropOffPreviousBarAngle;
            } inner, outer;
        } config;

        frc2::SequentialCommandGroup * mHighBarClimb = nullptr;
};