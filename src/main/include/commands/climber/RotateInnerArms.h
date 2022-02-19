#pragma once

#include "climber/climber.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class RotateInnerArmsCommand : public frc2::CommandHelper<frc2::CommandBase, RotateInnerArmsCommand> {
    public:
        RotateInnerArmsCommand(Climber* climber, double targetRotation);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        Climber * mClimber;
        double mTargetRotation;
};