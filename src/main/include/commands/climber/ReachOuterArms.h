#pragma once

#include "climber/OuterReach.h"
#include "PID.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ReachOuterArmsCommand : public frc2::CommandHelper<frc2::CommandBase, ReachOuterArmsCommand> {
    public:
        ReachOuterArmsCommand(ClimberOuterReach * outerArms, double targetPosition);
        ReachOuterArmsCommand(ClimberOuterReach * outerArms, double targetPosition, PID const & pid1, PID const & pid2);

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
        ClimberOuterReach * mOuterArms;
        double mTargetPosition;

        /*
        p, i, d, feed-forward, acceptableError,
        minOutput = -1.0, maxOutput = 1.0, izone = INFINITY
        */
        PID mPid1 { 0.2, 0.0, 0.0, 0.04, 0.05, -1.0, 1.0 };
        PID mPid2 { 0.2, 0.0, 0.0, 0.04, 0.05, -1.0, 1.0 };
};
