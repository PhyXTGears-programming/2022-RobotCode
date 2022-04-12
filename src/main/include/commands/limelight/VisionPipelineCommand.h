#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "limelight/LimelightSubsystem.h"

class VisionPipelineCommand : public frc2::CommandHelper<frc2::CommandBase, VisionPipelineCommand>
{
public:
    VisionPipelineCommand(LimelightSubsystem * limelightSubsystem);

    void Initialize() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    LimelightSubsystem  * mLimelightSubsystem = nullptr;
};