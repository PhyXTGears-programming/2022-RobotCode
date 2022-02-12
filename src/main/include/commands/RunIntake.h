#pragma once

#include "intake/intake.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class RunIntakeCommand : public frc2::CommandHelper<frc2::CommandBase, RunIntakeCommand> {
public:
    RunIntakeCommand(Intake* intake);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    Intake* mIntake;
};