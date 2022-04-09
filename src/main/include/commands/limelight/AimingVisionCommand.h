#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "limelight/LimelightSubsystem.h"

class AimingVisionCommand : public frc2::CommandHelper<frc2::CommandBase, AimingVisionCommand>
{
public:
    AimingVisionCommand(LimelightSubsystem * limelightSubsystem);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    LimelightSubsystem  * mLimelightSubsystem = nullptr;
};