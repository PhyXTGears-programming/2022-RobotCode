// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <frc2/command/CommandScheduler.h>

#include "RobotCompileModes.h"

#ifdef ROBOTCMH_PID_TUNING_MODE
#include "drivetrain/drivetrain.h"
static Drivetrain drivetrain;
#endif

void Robot::RobotInit() {
    #ifdef ROBOTCMH_TESTING_MODE
    #warning (In robot.cpp, testing mode is enabled)
    frc::SmartDashboard::PutString("Robot mode", "TESTING, RobotCompileModes.h");
    #endif

    std::shared_ptr<cpptoml::table> toml = LoadConfig("/home/lvuser/deploy/config.toml");

    operatorController = new frc::XboxController(interfaces::kXBoxOperator);

    mInnerReach = new ClimberInnerReach(toml->get_table("climber"));
    mInnerRotate = new ClimberInnerRotate(toml->get_table("climber"));
    mOuterReach = new ClimberOuterReach(toml->get_table("climber"));
    mOuterRotate = new ClimberOuterRotate(toml->get_table("climber"));
    mIntake = new Intake(toml->get_table("intake"));
    mShooter = new Shooter(toml->get_table("shooter"));
    mSwerveDrive = new SwerveDrive(false);

    mClimbMidbarOnly = new ClimbMidBarOnly(mInnerReach, mInnerRotate, toml->get_table_qualified("command.climb.midbar"));
    mHighClimb = new HighBarClimb(mIntake, mInnerReach, mInnerRotate, mOuterReach, mOuterRotate, toml->get_table_qualified("cycleCommand"));
    mTraversalClimb = new TraversalClimb(mIntake, mInnerReach, mInnerRotate, mOuterReach, mOuterRotate, toml->get_table_qualified("cycleCommand"));

    mManualRetractOuterArms = new frc2::FunctionalCommand(
        [&]() {},
        [&]() {
            bool isOuter1NearTarget = mOuterReach->isMotor1NearTarget(0.0);
            bool isOuter2NearTarget = mOuterReach->isMotor2NearTarget(0.0);
            bool isInner1NearTarget = mInnerReach->isMotor1NearTarget(0.0);
            bool isInner2NearTarget = mInnerReach->isMotor2NearTarget(0.0);

            if (isOuter1NearTarget) {
                mOuterReach->stop1();
            } else {
                mOuterReach->run1(-0.4);
            }

            if (isOuter2NearTarget) {
                mOuterReach->stop2();
            } else {
                mOuterReach->run2(-0.4);
            }

            if (isInner1NearTarget) {
                mInnerReach->stop1();
            } else {
                mInnerReach->run1(-0.4);
            }

            if (isInner2NearTarget) {
                mInnerReach->stop2();
            } else {
                mInnerReach->run2(-0.4);
            }
        },
        [&](bool) {
            mOuterReach->stop1();
            mOuterReach->stop2();
            mInnerReach->stop1();
            mInnerReach->stop2();
        },
        [&]() { return mOuterReach->getMotor1Position() < 1.0
            && mOuterReach->getMotor2Position() < 1.0
            && mInnerReach->getMotor1Position() < 1.0
            && mInnerReach->getMotor2Position() < 1.0; },
        { mOuterReach, mInnerReach }
    );

    mManualExtendOuterArms = new frc2::FunctionalCommand(
        [&]() {},
        [&]() {
            bool isOuter1NearTarget = mOuterReach->isMotor1NearTarget(20.0);
            bool isOuter2NearTarget = mOuterReach->isMotor2NearTarget(20.0);
            bool isInner1NearTarget = mInnerReach->isMotor1NearTarget(20.0);
            bool isInner2NearTarget = mInnerReach->isMotor2NearTarget(20.0);

            if (isOuter1NearTarget) {
                mOuterReach->stop1();
            } else {
                mOuterReach->run1(0.4);
            }

            if (isOuter2NearTarget) {
                mOuterReach->stop2();
            } else {
                mOuterReach->run2(0.4);
            }

            if (isInner1NearTarget) {
                mInnerReach->stop1();
            } else {
                mInnerReach->run1(0.4);
            }

            if (isInner2NearTarget) {
                mInnerReach->stop2();
            } else {
                mInnerReach->run2(0.4);
            }
        },
        [&](bool) {
            mOuterReach->stop1();
            mOuterReach->stop2();
            mInnerReach->stop1();
            mInnerReach->stop2();
        },
        [&]() { return mOuterReach->getMotor1Position() > 20.0
            && mOuterReach->getMotor2Position() > 20.0
            && mInnerReach->getMotor1Position() > 20.0
            && mInnerReach->getMotor2Position() > 20.0; },
        { mOuterReach, mInnerReach }
    );

    mRetractInnerArms = new RetractInnerArmsCommand {mInnerReach, 1.0};
    mExtendInnerArms = new ExtendInnerArmsCommand {mInnerReach, 10.0};
    mRetractOuterArms = new RetractOuterArmsCommand {mOuterReach, 1.0};
    mExtendOuterArms = new ExtendOuterArmsCommand {mOuterReach, 10.0};
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

    static int resyncCounter = 25;
    if (0 == resyncCounter) {
        resyncCounter = 25;
        // mSwerveDrive->synchronizeTurnEncoders();
    }
    resyncCounter--;
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
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
    //Climber
    if (0 == operatorController->GetPOV()) { // Check up button
        mClimbMidbarOnly->mReachMidBar->Schedule();
    }
    
    if (270 == operatorController->GetPOV()) { // Check left button
        if (!mHighClimb->IsScheduled()) {
            mHighClimb->Schedule();
        }
    } else {

    }

    if (180 == operatorController->GetPOV()) { // Check down button
      mClimbMidbarOnly->mClimbMidBar->Schedule();
    }

    if (90 == operatorController->GetPOV()) { // Check right button
        if (!mTraversalClimb->IsScheduled()) {
            mTraversalClimb->Schedule();
        }
    } else {
        
    }

    double opY = -operatorController->GetLeftY();
    opY = fabs(opY) < 0.3 ? 0.0 : opY;
    if (opY < 0.0) {
        mManualRetractOuterArms->Schedule();
    } else if (opY > 0.0) {
        mManualExtendOuterArms->Schedule();
    } else {
        if (mManualRetractOuterArms->IsScheduled()) {
            mManualRetractOuterArms->Cancel();
        } else if (mManualExtendOuterArms->IsScheduled()) {
            mManualExtendOuterArms->Cancel();
        }
    }

    // Climber test
    if (operatorController->GetXButtonPressed()) {
        mRetractInnerArms->Schedule();
    } else if (operatorController->GetYButtonPressed()) {
        mExtendInnerArms->Schedule();
    }

    if (operatorController->GetAButtonPressed()) {
        mRetractOuterArms->Schedule();
    } else if (operatorController->GetBButtonPressed()) {
        mExtendOuterArms->Schedule();
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
