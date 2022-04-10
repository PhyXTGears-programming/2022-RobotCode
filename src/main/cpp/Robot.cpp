// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <frc2/command/CommandScheduler.h>

#include "RobotCompileModes.h"

using namespace std::literals::string_view_literals;

constexpr std::string_view DASH_USE_OUTER_REACH_TEST_COMMAND = "Enable Outer Reach Test Command"sv;
constexpr std::string_view DASH_OUTER_REACH_TARGET = "Outer Reach Target Position"sv;
constexpr std::string_view DASH_OUTER_REACH_ACTIVATE = "Activate Outer Reach Command"sv;

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

    // Need to Put a value, first, in order to create an editable field on the dashboard.
    frc::SmartDashboard::PutBoolean(DASH_USE_OUTER_REACH_TEST_COMMAND, false);
    frc::SmartDashboard::PutNumber(DASH_OUTER_REACH_TARGET, 0.0);
    frc::SmartDashboard::PutBoolean(DASH_OUTER_REACH_ACTIVATE, false);
}

template <class T>
void notifyOnChange(T value, T & prevValue, std::function<void(T)> onChange) {
    if (value != prevValue) {
        prevValue = value;
        onChange(value);
    }
}

void updateDashboardBool(std::string_view name, bool & prevValue, std::function<void(bool)> onChange) {
    bool value = frc::SmartDashboard::GetBoolean(name, false);
    notifyOnChange(value, prevValue, onChange);
}

void updateDashboardNumber(std::string_view name, double & prevValue, std::function<void(double)> onChange) {
    double value = frc::SmartDashboard::GetNumber(name, 0.0);
    notifyOnChange(value, prevValue, onChange);
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

    frc::SmartDashboard::PutData(&frc2::CommandScheduler::GetInstance());

    {
        // Use dashboard to control outer arm reach.

        // We'll create a new command as the user changes parameters on the dashboard (e.g. target position).
        static ReachOuterArmsCommand * outerReachCommand = nullptr;

        // Allow user to disable the command.  Can be used to prevent accidental activation.  Currently not used in code, yet.
        static bool useOuterReachTestCommand = frc::SmartDashboard::GetBoolean(DASH_USE_OUTER_REACH_TEST_COMMAND, false);
        updateDashboardBool(DASH_USE_OUTER_REACH_TEST_COMMAND, useOuterReachTestCommand, [](bool value) {
            std::cout << "use outer reach command? " << (value ? "yes" : "no") << std::endl;
        });

        // Allow user to modify target position.
        static double outerReachTarget = frc::SmartDashboard::GetNumber(DASH_OUTER_REACH_TARGET, 0.0);
        updateDashboardNumber(DASH_OUTER_REACH_TARGET, outerReachTarget, [&](double value) {
            // Ensure value has a safe range.
            value = std::clamp(value, 2.0, 26.0);   // These limits try to protect from overshooting.  Max range [0, 28].

            if (nullptr != outerReachCommand) {
                if (outerReachCommand->IsScheduled()) {
                    outerReachCommand->Cancel();
                    std::cout << "Canceled current outer reach command" << std::endl;
                }
                delete outerReachCommand;
            }

            outerReachCommand = new ReachOuterArmsCommand(mOuterReach, value);
            std::cout << "Created new outer reach command" << std::endl;

            // Update dashboard with new value, if limited.
            frc::SmartDashboard::PutNumber(DASH_OUTER_REACH_TARGET, value);
        });

        // Allow user to start the command.
        static bool activateCommand = frc::SmartDashboard::GetBoolean(DASH_OUTER_REACH_ACTIVATE, false);
        updateDashboardBool(DASH_OUTER_REACH_ACTIVATE, activateCommand, [&](bool isActive) {
            // Do nothing when flag when false.
            if (isActive) {
                if (nullptr != outerReachCommand) {
                    // If a command is loaded...
                    if (! outerReachCommand->IsScheduled()) {
                        // ...and not scheduled, then start the command.
                        outerReachCommand->Schedule();
                        std::cout << "Scheduled outer reach command" << std::endl;
                    } else {
                        std::cout << "Command for outer reach already scheduled" << std::endl;
                    }
                } else {
                    std::cout << "No command for outer reach available.  Configure outer reach settings first." << std::endl;
                }

                // Reset dashboard checkbox back to empty, so use knows they can click to activate again.
                frc::SmartDashboard::PutBoolean(DASH_OUTER_REACH_ACTIVATE, false);
            }
        });
    }
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
