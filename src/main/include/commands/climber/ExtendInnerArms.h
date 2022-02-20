#pragma once

#include "climber/climber.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ExtendInnerArmsCommand : public frc2::CommandHelper<frc2::CommandBase, ExtendInnerArmsCommand> {
    public:
        ExtendInnerArmsCommand(Climber* climber, double targetExtension);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        Climber * mClimber;
        double mTargetExtension;
};