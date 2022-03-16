#pragma once

#include "climber/OuterReach.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ReachOuterArmsCommand : public frc2::CommandHelper<frc2::CommandBase, ReachOuterArmsCommand> {
    public:
        ReachOuterArmsCommand(ClimberOuterReach * outerArms, double targetPosition);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimberOuterReach * mOuterArms;
        double mTargetPosition;
};