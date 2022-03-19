#pragma once

#include "climber/InnerReach.h"
#include "climber/OuterReach.h"
#include "RetractInnerArms.h"
#include "RetractOuterArms.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ManualExtendCommand : public frc2::CommandHelper<frc2::CommandBase, ManualExtendCommand> {
    public:
        ManualExtendCommand(ClimberInnerReach * innerArms);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimberInnerReach * mInnerArms;
        const double kTargetExtension = 10.0;
        const double kRestingExtension = 0.0;

        RetractInnerArmsCommand * mRetract = nullptr; 
};