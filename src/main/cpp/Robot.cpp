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

frc2::FunctionalCommand * makeInnerReachCommand(
    double limitPosition,
    double speed,
    ClimberInnerReach * innerReach,
    std::function<bool(double, double)>
);

frc2::FunctionalCommand * makeOuterReachCommand(
    double limitPosition,
    double speed,
    ClimberOuterReach * outerReach,
    std::function<bool(double, double)>
);

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

    mClimbMidbarOnly = new ClimbMidBarOnly(mInnerReach, mInnerRotate, toml->get_table_qualified("command.climb.midbar"));
    mHighClimb = new HighBarClimb(mIntake, mInnerReach, mInnerRotate, mOuterReach, mOuterRotate, toml->get_table_qualified("cycleCommand"));
    mTraversalClimb = new TraversalClimb(mIntake, mInnerReach, mInnerRotate, mOuterReach, mOuterRotate, toml->get_table_qualified("cycleCommand"));

    mManualRetractInnerArms = makeInnerReachCommand(1.0, -0.6, mInnerReach, [](double limit, double pos) { return pos <= limit; });
    mManualExtendInnerArms = makeInnerReachCommand(20.0, 0.4, mInnerReach, [](double limit, double pos) { return pos >= limit; });

    mManualRetractOuterArms = makeOuterReachCommand(1.0, -0.6, mOuterReach, [](double limit, double pos) { return pos <= limit; });
    mManualExtendOuterArms = makeOuterReachCommand(20.0, 0.4, mOuterReach, [](double limit, double pos) { return pos >= limit; });


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

    // Manually operate inner arm reach with left analog Y.
    double leftY = -operatorController->GetLeftY();      // Convert (-1 is up, +1 is down) to (+1 up, -1 down).
    leftY = fabs(leftY) < 0.3 ? 0.0 : leftY;
    if (leftY < 0.0) {
        mManualRetractOuterArms->Schedule();
    } else if (leftY > 0.0) {
        mManualExtendOuterArms->Schedule();
    } else {
        if (mManualRetractOuterArms->IsScheduled()) {
            mManualRetractOuterArms->Cancel();
        } else if (mManualExtendOuterArms->IsScheduled()) {
            mManualExtendOuterArms->Cancel();
        }
    }

    // Manually operate outer arm reach with right analog Y.
    double rightY = -operatorController->GetRightY();    // Convert (-1 is up, +1 is down) to (+1 up, -1 down).
    rightY = fabs(rightY) < 0.3 ? 0.0 : rightY;
    if (rightY < 0.0) {
        mManualRetractInnerArms->Schedule();
    } else if (rightY > 0.0) {
        mManualExtendInnerArms->Schedule();
    } else {
        if (mManualRetractInnerArms->IsScheduled()) {
            mManualRetractInnerArms->Cancel();
        } else if (mManualExtendInnerArms->IsScheduled()) {
            mManualExtendInnerArms->Cancel();
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


frc2::FunctionalCommand * makeInnerReachCommand(
    double limitPosition,
    double speed,
    ClimberInnerReach * innerReach,
    std::function<bool(double limit, double position)> isAtLimit
) {
    return new frc2::FunctionalCommand(
        [=]() {},
        [=]() {
            bool is1NearTarget = isAtLimit(limitPosition, innerReach->getMotor1Position());
            bool is2NearTarget = isAtLimit(limitPosition, innerReach->getMotor2Position());

            if (is1NearTarget) {
                innerReach->stop1();
            } else {
                innerReach->run1(speed);
            }

            if (is2NearTarget) {
                innerReach->stop2();
            } else {
                innerReach->run2(speed);
            }
        },
        [=](bool) {
            innerReach->stop1();
            innerReach->stop2();
        },
        [=]() {
            return isAtLimit(limitPosition, innerReach->getMotor1Position())
                && isAtLimit(limitPosition, innerReach->getMotor2Position());
        },
        { innerReach }
    );
}

frc2::FunctionalCommand * makeOuterReachCommand(
    double limitPosition,
    double speed,
    ClimberOuterReach * outerReach,
    std::function<bool(double limit, double position)> isAtLimit
) {
    return new frc2::FunctionalCommand(
        [=]() {},
        [=]() {
            bool is1NearTarget = isAtLimit(limitPosition, outerReach->getMotor1Position());
            bool is2NearTarget = isAtLimit(limitPosition, outerReach->getMotor2Position());

            if (is1NearTarget) {
                outerReach->stop1();
            } else {
                outerReach->run1(speed);
            }

            if (is2NearTarget) {
                outerReach->stop2();
            } else {
                outerReach->run2(speed);
            }
        },
        [=](bool) {
            outerReach->stop1();
            outerReach->stop2();
        },
        [=]() {
            return isAtLimit(limitPosition, outerReach->getMotor1Position())
                && isAtLimit(limitPosition, outerReach->getMotor2Position());
        },
        { outerReach }
    );
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
