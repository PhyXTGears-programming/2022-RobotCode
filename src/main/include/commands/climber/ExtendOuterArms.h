#pragma once

#include "climber/OuterReach.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ExtendOuterArmsCommand : public frc2::CommandHelper<frc2::CommandBase, ExtendOuterArmsCommand> {
    public:
        ExtendOuterArmsCommand(ClimberOuterReach * outerArms, double targetExtension);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimberOuterReach * mOuterArms;
        double mTargetExtension;
};