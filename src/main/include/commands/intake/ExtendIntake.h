#pragma once

#include "intake/intake.h"

#include "frc2/command/CommandBase.h"
#include "frc2/command/CommandHelper.h"

class ExtendIntakeCommand : public frc2::CommandHelper<frc2::CommandBase, ExtendIntakeCommand> {
public:
    ExtendIntakeCommand(Intake * intake);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:

    Intake* mIntake;

};