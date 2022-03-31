#pragma once

#include "climber/OuterReach.h"

#include <frc/controller/PIDController.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ReachOuterArmsCommand : public frc2::CommandHelper<frc2::CommandBase, ReachOuterArmsCommand> {
    public:
        ReachOuterArmsCommand(ClimberOuterReach * outerArms, double targetPosition);

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

        frc2::PIDController mPid1 { 0.2, 0.000, 0.0 };
        frc2::PIDController mPid2 { 0.2, 0.000, 0.0 };

        double mFF = 0.03;
};
