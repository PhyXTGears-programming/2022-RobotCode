#pragma once


#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "limelight/limelight.h"
#include "commands/limelight/VisionPipelineCommand.h"

#include "drivetrain-swerve/SwerveDrive.h"

class AimAutoCommand : public frc2::CommandHelper<frc2::CommandBase, AimAutoCommand> {
public:
    AimAutoCommand(double strafe, double forewards, double acceptableAngleError, limelight * limelight, VisionPipelineCommand * visionPipelineCommand, SwerveDrive * swerveDrive);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    double kStrafe;
    double kForewards;
    double kAcceptableAngleError;
    limelight * mLimelight = nullptr;
    VisionPipelineCommand * mVisionPipelineCommand = nullptr;
    SwerveDrive * mSwerveDrive = nullptr;
};