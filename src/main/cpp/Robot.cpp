// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "RobotCompileModes.h"


#ifdef ROBOTCMH_PID_TUNING_MODE
#include "drivetrain/drivetrain.h"
static Drivetrain drivetrain;
#endif

void Robot::RobotInit()
{
#ifdef ROBOTCMH_TESTING_MODE
#warning(In robot.cpp, testing mode is enabled)
    frc::SmartDashboard::PutString("Robot mode", "TESTING, RobotCompileModes.h");
#endif

    std::shared_ptr<cpptoml::table> toml = LoadConfig("/home/lvuser/deploy/config.toml");

    driverController = new frc::XboxController(interfaces::kXBoxDriver);
    operatorController = new frc::XboxController(interfaces::kXBoxOperator);
  
    mClimber = new Climber(toml->get_table("climber"));
    mIntake = new Intake(toml->get_table("intake"));
    mShooter = new Shooter(toml->get_table("shooter"));
    mSwerveDrive = new SwerveDrive(false);
   
    mDriveTeleopCommand = new AltDriveTeleopCommand(driverController, mSwerveDrive);
    mClimbMidbarOnly = new ClimbMidBarOnly(mClimber, toml->get_table_qualified("command.climb.midbar"));
    mRunIntakeCommand = new RunIntakeCommand(mIntake);
    mShootCommand = new ShootCommand(mShooter);

    mManualRetractInnerArms = new frc2::FunctionalCommand(
        [&]() {},
        [&]() {
            bool isInner1NearTarget = mClimber->isInner1NearTarget(0.0);
            bool isInner2NearTarget = mClimber->isInner2NearTarget(0.0);

            if (isInner1NearTarget) {
                mClimber->stopInner1();
            } else {
                mClimber->runInner1(0.2);
            }

            if (isInner2NearTarget) {
                mClimber->stopInner2();
            } else {
                mClimber->runInner2(0.2);
            }
        },
        [&](bool) {
            mClimber->stopInner1();
            mClimber->stopInner2();
        },
        [&]() { return mClimber->isInner1NearTarget(0.0) || mClimber->isInner2NearTarget(0.0); },
        { mClimber }
    );

    mManualExtendInnerArms = new frc2::FunctionalCommand(
        [&]() {},
        [&]() {
            bool isInner1NearTarget = mClimber->isInner1NearTarget(20.0);
            bool isInner2NearTarget = mClimber->isInner2NearTarget(20.0);

            if (isInner1NearTarget) {
                mClimber->stopInner1();
            } else {
                mClimber->runInner1(-0.2);
            }

            if (isInner2NearTarget) {
                mClimber->stopInner2();
            } else {
                mClimber->runInner2(-0.2);
            }
        },
        [&](bool) {
            mClimber->stopInner1();
            mClimber->stopInner2();
        },
        [&]() { return mClimber->isInner1NearTarget(20.0) || mClimber->isInner2NearTarget(20.0); },
        { mClimber }
    );


    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoShootAndDrive, kAutoShootAndDrive);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    mShootAndDrive = new frc2::SequentialCommandGroup {
      ShootCommand {mShooter}.WithTimeout(1_s),
      frc2::FunctionalCommand { // drive backwards
        [](){},
        [&](){
          mSwerveDrive->setMotion(0, -0.5, 0);
        },
        [&](bool _interrupted){ 
          mSwerveDrive->setMotion(0, 0, 0); //stop swerve
        }, 
        [](){ return false; },
        {mSwerveDrive}
      }.WithTimeout(2_s)
    };
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector",
    //     kAutoNameDefault);
    std::cout << "Auto selected: " << m_autoSelected << std::endl;

    if (m_autoSelected == kAutoShootAndDrive) {
        mShootAndDrive->Schedule();
    } else {
        // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic()
{
    if (m_autoSelected == kAutoNameCustom) {
        // Custom Auto goes here
    } else {
        // Default Auto goes here
    }
}

void Robot::TeleopInit() {
    mDriveTeleopCommand->Schedule();
}

void Robot::TeleopPeriodic()
{
#ifdef ROBOTCMH_PID_TUNING_MODE
    drivetrain.tunePIDNetworktables();
#endif
    //Shooter
    if (operatorController->GetXButtonPressed()) {
        mShootCommand->Schedule();
    } else if(operatorController->GetXButtonReleased()) {
        mShootCommand->Cancel();
    }

    //Intake
    if (operatorController->GetAButtonPressed()) {
        mRunIntakeCommand->Schedule();
    } else if(operatorController->GetAButtonReleased()) {
        mRunIntakeCommand->Cancel();
    }

    //Climber
    if (0 == operatorController->GetPOV()) { // Check up button
        mClimbMidbarOnly->mReachMidBar->Schedule();
    }
    
    if (90 == operatorController->GetPOV()) { // Check right button

    } else {

    }

    if (180 == operatorController->GetPOV()) { // Check down button
      mClimbMidbarOnly->mClimbMidBarAndLock->Schedule();
    }

    if (270 == operatorController->GetPOV()) { // Check left button

    } else {
        
    }

    // FIXME: Hack to allow operator to manually (and slowly) move inner climb
    // arms if no other command is running.
    double opY = operatorController->GetLeftY();
    opY = fabs(opY) < 0.3 ? 0.0 : opY;
    if (opY < 0.0) {
        mManualRetractInnerArms->Schedule();
    } else if (opY > 0.0) {
        mManualExtendInnerArms->Schedule();
    } else {
        if (mManualRetractInnerArms->IsScheduled()) {
            mManualRetractInnerArms->Cancel();
        } else if (mManualExtendInnerArms->IsScheduled()) {
            mManualExtendInnerArms->Cancel();
        }
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

std::shared_ptr<cpptoml::table> Robot::LoadConfig(std::string path)
{
    try
    {
        return cpptoml::parse_file(path);
    }
    catch (cpptoml::parse_exception &ex)
    {
        std::cerr << "Error loading config file: " << path << std::endl
                  << ex.what() << std::endl;
        exit(1);
    }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
