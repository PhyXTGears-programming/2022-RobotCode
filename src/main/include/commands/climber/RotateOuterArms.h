#pragma once

#include "climber/OuterRotate.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <cmath>

class RotateOuterArmsCommand : public frc2::CommandHelper<frc2::CommandBase, RotateOuterArmsCommand> {
    public:
        RotateOuterArmsCommand(ClimberOuterRotate * outerArms, double targetAngle);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimberOuterRotate * mOuterArms;
        double mTargetAngle;
};