// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/StartEndCommand.h>

#include <frc/XboxController.h>

#include "constants/interfaces.h"

#include "cpptoml.h"

#include "climber/InnerReach.h"
#include "climber/InnerRotate.h"
#include "climber/OuterReach.h"
#include "climber/OuterRotate.h"
#include "drivetrain-swerve/SwerveDrive.h"
#include "intake/intake.h"
#include "shooter/shooter.h"

#include "commands/climber/ClimbMidBarOnly.h"
#include "commands/drivetrain-swerve/AltDriveTeleopCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "commands/intake/RunIntake.h"
#include "commands/intake/ExtendIntake.h"
#include "commands/intake/RetractIntake.h"

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
    const std::string kAutoDriveAndShoot = "Drive and Shoot";
    std::string m_autoSelected;

    frc::XboxController * driverController = nullptr;
    frc::XboxController * operatorController = nullptr;

    ClimberInnerReach * mInnerReach = nullptr;
    ClimberInnerRotate * mInnerRotate = nullptr;
    ClimberOuterReach * mOuterReach = nullptr;
    ClimberOuterRotate * mOuterRotate = nullptr;
    Intake *mIntake = nullptr;
    Shooter *mShooter = nullptr;
    SwerveDrive * mSwerveDrive = nullptr;
    
    AltDriveTeleopCommand * mDriveTeleopCommand = nullptr;
    ClimbMidBarOnly * mClimbMidbarOnly = nullptr;
    ShootCommand *mShootCommand = nullptr;
    
    RunIntakeCommand *mRunIntakeCommand = nullptr;
    RetractIntakeCommand *mRetractIntakeCommand = nullptr;
    ExtendIntakeCommand *mExtendIntakeCommand = nullptr;


    frc2::FunctionalCommand *mManualRetractInnerArms = nullptr;
    frc2::FunctionalCommand *mManualExtendInnerArms = nullptr;

    frc2::StartEndCommand *mShootNear = nullptr;
    frc2::StartEndCommand *mShootFar = nullptr;
    frc2::StartEndCommand *mShootLowHub = nullptr;
    frc2::StartEndCommand *mShootReverse = nullptr;
    frc2::StartEndCommand *mShootAuto = nullptr;

    frc2::SequentialCommandGroup * mDriveAndShoot = nullptr;
};

// there is a hidden forg somewhere in the robot's code.
//Where does the Prog Frog's Virtual Identity Clone to Infiltrate Your Programs and Scripts Maliciously and With Evil Intent (PFVICIYPSMWEI) seem to be now?