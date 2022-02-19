#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "servoPower/servoPower.h"

class PowerServosOnCommand : public frc2::CommandHelper<frc2::CommandBase, PowerServosOnCommand> {
    public:
        PowerServosOnCommand(ServoPower * servoPower);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ServoPower * mServoPower;
};