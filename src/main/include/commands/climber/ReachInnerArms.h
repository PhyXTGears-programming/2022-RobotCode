#pragma once

#include "climber/InnerReach.h"
#include "PID.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ReachInnerArmsCommand : public frc2::CommandHelper<frc2::CommandBase, ReachInnerArmsCommand> {
    public:
        ReachInnerArmsCommand(ClimberInnerReach * outerArms, double targetPosition);
        ReachInnerArmsCommand(ClimberInnerReach * outerArms, double targetPosition, PID const & pid1, PID const & pid2);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

        void SetPid(double p, double i, double d, double ff);
        void SetP(double p);
        void SetI(double i);
        void SetD(double d);
        void SetFF(double ff);

    private:
        ClimberInnerReach * mInnerArms;
        double mTargetPosition;

        /*
        p, i, d, feed-forward, acceptableError,
        minOutput = -1.0, maxOutput = 1.0, izone = INFINITY
        */
        PID mPid1 { 0.5, 0.0, 0.0, 0.04, 0.05, -1.0, 1.0 };
        PID mPid2 { 0.5, 0.0, 0.0, 0.04, 0.05, -1.0, 1.0 };
};
