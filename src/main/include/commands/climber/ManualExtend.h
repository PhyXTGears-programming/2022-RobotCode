#pragma once

#include "climber/climber.h"
#include "RetractInnerArms.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ManualExtendCommand : public frc2::CommandHelper<frc2::CommandBase, ManualExtendCommand> {
    public:
        ManualExtendCommand(Climber* climber);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        Climber * mClimber;
        const double kTargetExtension = 10.0;
        const double kRestingExtension = 0.0;

        RetractInnerArmsCommand * mRetract = nullptr; 
};