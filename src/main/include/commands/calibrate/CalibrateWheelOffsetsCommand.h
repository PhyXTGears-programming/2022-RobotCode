#pragma once

#include "drivetrain-swerve/SwerveDrive.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <optional>

class CalibrateWheelOffsetsCommand : public frc2::CommandHelper<frc2::CommandBase, CalibrateWheelOffsetsCommand> {
public:
    CalibrateWheelOffsetsCommand(SwerveDrive * drive, std::function<double ()> driveAxis);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    frc2::SequentialCommandGroup* mProcedure = nullptr;

    SwerveDrive* mSwerve = nullptr;

    std::optional<WheelOffsets> mWheelOffsets;
};