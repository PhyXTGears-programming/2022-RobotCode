#pragma once

#include "climber/InnerRotate.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <cmath>

#define MIN_SPEED 0.15
#define MAX_SPEED 0.2

class RotateInnerArmsCommand : public frc2::CommandHelper<frc2::CommandBase, RotateInnerArmsCommand> {
    public:
        RotateInnerArmsCommand(
            ClimberInnerRotate * innerArms,
            double targetAngle,
            double minSpeed = MIN_SPEED,
            double maxSpeed = MAX_SPEED
        );

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimberInnerRotate * mInnerArms;
        double mTargetAngle;
        double mMinSpeed;
        double mMaxSpeed;
};