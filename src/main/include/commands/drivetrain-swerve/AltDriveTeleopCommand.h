#pragma once

#include "drivetrain-swerve/SwerveDrive.h"
#include "limelight/limelight.h"
#include "commands/limelight/VisionPipelineCommand.h"
#include <functional>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

class AltDriveTeleopCommand : public frc2::CommandHelper<frc2::CommandBase, AltDriveTeleopCommand> {
public:
    AltDriveTeleopCommand(frc::XboxController* driverController, SwerveDrive * _swerveDrive, limelight * limelight, VisionPipelineCommand * VisionPipelineCommand);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    frc::XboxController* mJoystick = nullptr;

    SwerveDrive * swerveDrive = nullptr;

    limelight * mLimelight = nullptr;

    VisionPipelineCommand * mVisionPipelineCommand = nullptr;
};