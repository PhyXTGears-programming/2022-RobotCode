#pragma once

#include "climber/InnerRotate.h"
#include "PID.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#define MIN_SPEED 0.15
#define MAX_SPEED 0.2

class RotateInnerArmsCommand : public frc2::CommandHelper<frc2::CommandBase, RotateInnerArmsCommand> {
    public:
        RotateInnerArmsCommand(ClimberInnerRotate * innerArms, double targetAngle, PID & pid);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimberInnerRotate * mInnerArms;
        double mTargetAngle;

        PID mPid;
};