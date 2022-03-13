#pragma once

#include "climber/OuterRotate.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <cmath>

class RotateOuterArmsCommand : public frc2::CommandHelper<frc2::CommandBase, RotateOuterArmsCommand> {
    public:
        RotateOuterArmsCommand(OuterRotate * outerArms, double targetAngle);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        OuterRotate * mOuterArms;
        double mTargetAngle;
};