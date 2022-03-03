// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/XboxController.h>

#include "constants/interfaces.h"

#include "cpptoml.h"

#include "drivetrain-swerve/SwerveDrive.h"
#include "intake/intake.h"
#include "shooter/shooter.h"

#include "commands/drivetrain-swerve/AltDriveTeleopCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "commands/intake/RunIntake.h"

class Robot : public frc::TimedRobot
{
public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;

    std::shared_ptr<cpptoml::table> LoadConfig(std::string path);

private:
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

    frc::XboxController * driverController = nullptr;
    frc::XboxController * operatorController = nullptr;

    Intake *mIntake = nullptr;
    Shooter *mShooter = nullptr;
    SwerveDrive * mSwerveDrive = nullptr;
    
    AltDriveTeleopCommand * mDriveTeleopCommand = nullptr;
    ShootCommand *mShootCommand = nullptr;
    RunIntakeCommand *mRunIntakeCommand = nullptr;
};
