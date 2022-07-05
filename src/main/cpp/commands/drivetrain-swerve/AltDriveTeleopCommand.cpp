#include "commands/drivetrain-swerve/AltDriveTeleopCommand.h"

#include "drivetrain-swerve/SwerveDrive.h"
#include <functional>
#include <iostream>
#include <cmath>

#include "constants/constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "limelight/limelight.h"
#include "commands/limelight/VisionPipelineCommand.h"

#define JOYSTICK_DEADZONE 0.2       // (jcc) Tested Mar 3.  0.2 prevents wheels from steering wildly near deadzone.
#define MAKE_VALUE_FULL_RANGE(deadzonedInput) (1 / (1 - JOYSTICK_DEADZONE) * (deadzonedInput - std::copysign(JOYSTICK_DEADZONE, deadzonedInput)))
#define DEADZONE(input) ((std::abs(input) < JOYSTICK_DEADZONE) ? 0.0 : input)


#define TRIG_DEADZONE 0.1
#define TRIGGER_DEADZONE(input) ((std::abs(input) < TRIG_DEADZONE) ? 0.0 : input)

AltDriveTeleopCommand::AltDriveTeleopCommand(frc::XboxController *driverController, SwerveDrive * _swerveDrive, limelight * limelight, VisionPipelineCommand * visionPipelineCommand)
{
    AddRequirements(_swerveDrive);
    mLimelight = limelight;
    mVisionPipelineCommand = visionPipelineCommand;
    swerveDrive = _swerveDrive;
    mJoystick = driverController;
}

void AltDriveTeleopCommand::Initialize()
{
    
}

void AltDriveTeleopCommand::Execute()
{
    constexpr double MAX_SPEED = 0.75;
    double speed = MAX_SPEED - ((TRIGGER_DEADZONE(mJoystick->GetLeftTriggerAxis())) * 0.25);

    double x = mJoystick->GetLeftX();
    double y = -1 * mJoystick->GetLeftY();
    double r;
    if(mJoystick->GetAButton()){
        if(!mVisionPipelineCommand->IsScheduled()){
            mVisionPipelineCommand->Schedule();
        }
        r = mLimelight->PIDCalculate();
    } else {
        if(mVisionPipelineCommand->IsScheduled()){
            mLimelight->finishAim();
            mVisionPipelineCommand->Cancel();
        }
        r = -mJoystick->GetRightX();     // Invert RightX so left turns go left and not right.
        r = DEADZONE(r);
        r = r*r*r;
    }

    x = DEADZONE(x) * speed;
    y = DEADZONE(y) * speed;
    r = r * speed;
    swerveDrive->setMotion(x, y, r);
}

void AltDriveTeleopCommand::End(bool interrupted)
{

    std::cout << "interupted" << std::endl;
    //stopping code
}

bool AltDriveTeleopCommand::IsFinished()
{
    //finished code
    //never say it is finished because the drivetrain will never need to stop moving in teleop
    return false;
}