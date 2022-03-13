#pragma once

#include "climber/InnerReach.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class LockArmsCommand : public frc2::CommandHelper<frc2::CommandBase, LockArmsCommand> {
    public:
        LockArmsCommand(InnerReach * innerArms);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        InnerReach * mInnerArms;
};