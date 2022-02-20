#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "climber/climber.h"

class PowerServosOffCommand : public frc2::CommandHelper<frc2::CommandBase, PowerServosOffCommand> {
    public:
        PowerServosOffCommand(Climber * climber);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        Climber * mClimber;
};