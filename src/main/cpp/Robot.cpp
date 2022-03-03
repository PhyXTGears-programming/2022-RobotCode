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
    mIntake = new Intake(toml->get_table("intake"));

    driverController = new frc::XboxController(interfaces::kXBoxDriver);
    operatorController = new frc::XboxController(interfaces::kXBoxOperator};
  
    mShooter = new Shooter(toml->get_table("shooter"));
    mSwerveDrive = new SwerveDrive(false);
   
    mDriveTeleopCommand = new AltDriveTeleopCommand(driverController, mSwerveDrive);
    mRunIntakeCommand = new RunIntakeCommand(mIntake);
    mShootCommand = new ShootCommand(mShooter);

    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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

    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic()
{
    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
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
    if (operatorController.GetXButtonPressed()) 
    {
        mShootCommand->Schedule();
    }
    else if(operatorController.GetXButtonReleased())
    {
        mShootCommand->Cancel();
    }

    //Intake
    if (operatorController.GetAButtonPressed())
    {
        mRunIntakeCommand->Schedule();
    }
    else if(operatorController.GetAButtonReleased())
    {
        mRunIntakeCommand->Cancel();
    }

    //Climber
    if (operatorController.GetPOV(0)) // Check up button
    {

    }
    else
    {

    }
    
    if (operatorController.GetPOV(90)) // Check right button
    {

    }
    else
    {

    }

    if (operatorController.GetPOV(180)) // Check down button
    {

    }
    else
    {

    }

    if (operatorController.GetPOV(270)) // Check left button
    {

    }
    else
    {
        
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
