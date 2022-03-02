#pragma once

#include "drivetrain/drivetrain.h"
#include <functional>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

class DriveTeleopCommand : public frc2::CommandHelper<frc2::CommandBase, DriveTeleopCommand> {
public:
    DriveTeleopCommand(Drivetrain* drivetrain, frc::XboxController* driverController);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    void grabJoystickValues();
    double theeta(double x, double y);
    float cartToPolar(float inputX, float inputY);
    frc::XboxController* mJoystick = nullptr;
    double mJoystickAxis[4];
    Drivetrain* mDrivetrain = nullptr;

    double angle;
    double radius;
    double prevGoodAngle;
};