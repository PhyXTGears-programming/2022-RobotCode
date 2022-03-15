#pragma once

#include "climber/InnerRotate.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <cmath>

class RotateInnerArmsCommand : public frc2::CommandHelper<frc2::CommandBase, RotateInnerArmsCommand> {
    public:
        RotateInnerArmsCommand(ClimberInnerRotate * innerArms, double targetAngle);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimberInnerRotate * mInnerArms;
        double mTargetAngle;
};