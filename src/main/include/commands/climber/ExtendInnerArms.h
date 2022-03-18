#pragma once

#include "climber/InnerReach.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ExtendInnerArmsCommand : public frc2::CommandHelper<frc2::CommandBase, ExtendInnerArmsCommand> {
    public:
        ExtendInnerArmsCommand(ClimberInnerReach* innerArms, double targetExtension);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimberInnerReach * mInnerArms;
        double mTargetExtension;
};