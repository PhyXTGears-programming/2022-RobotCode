#pragma once
#include "shooter/shooter.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ShootCommand
    : public frc2::CommandHelper<frc2::CommandBase, ShootCommand> {
    public:
    
    explicit ShootCommand(Shooter* shooter);
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    private:
        Shooter* mShooter;


    };