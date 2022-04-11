// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "PID.h"

#include <iostream>
#include <frc2/command/CommandScheduler.h>

#include "RobotCompileModes.h"

using namespace std::literals::string_view_literals;

constexpr std::string_view DASH_USE_OUTER_REACH_TEST_COMMAND = "Enable Outer Reach Test Command"sv;
constexpr std::string_view DASH_OUTER_REACH_TARGET = "Outer Reach Target Position"sv;
constexpr std::string_view DASH_OUTER_REACH_ACTIVATE = "Activate Outer Reach Command"sv;

constexpr std::string_view DASH_USE_OUTER_ROTATION_TEST_COMMAND = "Enable Outer Rotation Test Command"sv;
constexpr std::string_view DASH_OUTER_ROTATE_TARGET = "Outer Rotate Target Rotation"sv;
constexpr std::string_view DASH_OUTER_ROTATE_ACTIVATE = "Activate Outer Rotate Command"sv;

constexpr std::string_view DASH_USE_INNER_REACH_TEST_COMMAND = "Enable Inner Reach Test Command"sv;
constexpr std::string_view DASH_INNER_REACH_TARGET = "Inner Reach Target Position"sv;
constexpr std::string_view DASH_INNER_REACH_ACTIVATE = "Activate Inner Reach Command"sv;

constexpr std::string_view DASH_USE_INNER_ROTATION_TEST_COMMAND = "Enable Inner Rotation Test Command"sv;
constexpr std::string_view DASH_INNER_ROTATE_TARGET = "Inner Rotate Target Rotation"sv;
constexpr std::string_view DASH_INNER_ROTATE_ACTIVATE = "Activate Inner Rotate Command"sv;

constexpr std::string_view DASH_SWING_USE_COMMAND = "Enable Swing Command"sv;
constexpr std::string_view DASH_SWING_INNER_REACH_TARGET = "Swing Reach In Target"sv;
constexpr std::string_view DASH_SWING_OUTER_REACH_TARGET = "Swing Reach Out Target"sv;
constexpr std::string_view DASH_SWING_ACTIVATE = "Activate Swing Command"sv;

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

    mInnerRotate->resetCurrentLimit();
    mOuterRotate->resetCurrentLimit();

    // Need to Put a value, first, in order to create an editable field on the dashboard.
    frc::SmartDashboard::PutBoolean(DASH_USE_OUTER_REACH_TEST_COMMAND, false);
    frc::SmartDashboard::PutNumber(DASH_OUTER_REACH_TARGET, 0.0);
    frc::SmartDashboard::PutBoolean(DASH_OUTER_REACH_ACTIVATE, false);

    frc::SmartDashboard::PutBoolean(DASH_USE_OUTER_ROTATION_TEST_COMMAND, false);
    frc::SmartDashboard::PutNumber(DASH_OUTER_ROTATE_TARGET, 0.0);
    frc::SmartDashboard::PutBoolean(DASH_OUTER_ROTATE_ACTIVATE, false);

    frc::SmartDashboard::PutBoolean(DASH_USE_INNER_REACH_TEST_COMMAND, false);
    frc::SmartDashboard::PutNumber(DASH_INNER_REACH_TARGET, 0.0);
    frc::SmartDashboard::PutBoolean(DASH_INNER_REACH_ACTIVATE, false);

    frc::SmartDashboard::PutBoolean(DASH_USE_INNER_ROTATION_TEST_COMMAND, false);
    frc::SmartDashboard::PutNumber(DASH_INNER_ROTATE_TARGET, 0.0);
    frc::SmartDashboard::PutBoolean(DASH_INNER_ROTATE_ACTIVATE, false);
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

        // Allow user to modify target position.
        static double outerReachTarget = frc::SmartDashboard::GetNumber(DASH_OUTER_REACH_TARGET, 0.0);
        updateDashboardNumber(DASH_OUTER_REACH_TARGET, outerReachTarget, [&](double value) {
            // Ensure value has a safe range.
            value = std::clamp(value, 1.0, 27.0);   // These limits try to protect from overshooting.  Max range [0, 28].

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
        updateDashboardBool(DASH_OUTER_REACH_ACTIVATE, activateCommand, [&](bool shallActivate) {
            bool isEnabled = frc::SmartDashboard::GetBoolean(DASH_USE_OUTER_REACH_TEST_COMMAND, false);

            // Do nothing when flag when not active OR command is disabled.
            if (isEnabled && shallActivate) {
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
            }

            // Reset dashboard checkbox back to empty, so use knows they can click to activate again.
            frc::SmartDashboard::PutBoolean(DASH_OUTER_REACH_ACTIVATE, false);
        });
    }

    {
        // Use dashboard to control outer arm rotation.

        // We'll create a new command as the user changes parameters on the dashboard (e.g. target position).
        static RotateOuterArmsCommand * outerRotateCommand = nullptr;

        static PID rotatePid { 0.0, 0.0, 0.0, 0.1, 0.5, -0.3, 0.3 };

        // Allow user to modify target position.
        static double outerRotateTarget = frc::SmartDashboard::GetNumber(DASH_OUTER_ROTATE_TARGET, 0.0);
        updateDashboardNumber(DASH_OUTER_ROTATE_TARGET, outerRotateTarget, [&](double target) {
            // Unsure what values constitute a safe range.  BE CAREFUL.

            if (nullptr != outerRotateCommand) {
                // Found an existing rotate command.
                if (outerRotateCommand->IsScheduled()) {
                    // Stop active rotate command.
                    outerRotateCommand->Cancel();
                    std::cout << "Canceled current outer rotate command" << std::endl;
                }
                // Delete object to free allocated memory to the OS.
                delete outerRotateCommand;
            }

            // Allocate a fresh command object.
            outerRotateCommand = new RotateOuterArmsCommand(mOuterRotate, target, rotatePid);
            std::cout << "Created new outer rotate command" << std::endl;

            // Update dashboard with new value, if limited.  Not necessary,
            // since target was clamped to a "safe" range.
            frc::SmartDashboard::PutNumber(DASH_OUTER_ROTATE_TARGET, target);
        });

        // Allow user to start the command.
        static bool activateCommand = frc::SmartDashboard::GetBoolean(DASH_OUTER_ROTATE_ACTIVATE, false);
        updateDashboardBool(DASH_OUTER_ROTATE_ACTIVATE, activateCommand, [&](bool shallActivate) {
            bool isEnabled = frc::SmartDashboard::GetBoolean(DASH_USE_OUTER_ROTATION_TEST_COMMAND, false);

            // Do nothing when flag is not active OR command is disabled.
            if (isEnabled && shallActivate) {
                if (nullptr != outerRotateCommand) {
                    // If a command is loaded...
                    if (! outerRotateCommand->IsScheduled()) {
                        // ...and not scheduled, then start the command.
                        outerRotateCommand->Schedule();
                        std::cout << "Scheduled outer rotate oommand" << std::endl;
                    } else {
                        std::cout << "Command for outer rotate already scheduled" << std::endl;
                    }
                } else {
                    std::cout << "No command for outer rotate available.  Configure outer rotate settings first." << std::endl;
                }
            } else if (shallActivate) {
                std::cout << "Command not enabled for outer rotate." << std::endl;
            }

            // Reset dashboard checkbox back to empty, so use knows they can click to activate again.
            frc::SmartDashboard::PutBoolean(DASH_OUTER_ROTATE_ACTIVATE, false);
        });
    }



    {
        // Use dashboard to control inner arm reach.

        // We'll create a new command as the user changes parameters on the dashboard (e.g. target position).
        static ReachInnerArmsCommand * innerReachCommand = nullptr;

        // Allow user to modify target position.
        static double innerReachTarget = frc::SmartDashboard::GetNumber(DASH_INNER_REACH_TARGET, 0.0);
        updateDashboardNumber(DASH_INNER_REACH_TARGET, innerReachTarget, [&](double value) {
            // Ensure value has a safe range.
            value = std::clamp(value, 1.0, 27.0);   // These limits try to protect from overshooting.  Max range [0, 28].

            if (nullptr != innerReachCommand) {
                if (innerReachCommand->IsScheduled()) {
                    innerReachCommand->Cancel();
                    std::cout << "Canceled current inner reach command" << std::endl;
                }
                delete innerReachCommand;
            }

            innerReachCommand = new ReachInnerArmsCommand(mInnerReach, value);
            std::cout << "Created new inner reach command" << std::endl;

            // Update dashboard with new value, if limited.
            frc::SmartDashboard::PutNumber(DASH_INNER_REACH_TARGET, value);
        });

        // Allow user to start the command.
        static bool activateCommand = frc::SmartDashboard::GetBoolean(DASH_INNER_REACH_ACTIVATE, false);
        updateDashboardBool(DASH_INNER_REACH_ACTIVATE, activateCommand, [&](bool shallActivate) {
            bool isEnabled = frc::SmartDashboard::GetBoolean(DASH_USE_INNER_REACH_TEST_COMMAND, false);

            // Do nothing when flag when not active OR command is disabled.
            if (isEnabled && shallActivate) {
                if (nullptr != innerReachCommand) {
                    // If a command is loaded...
                    if (! innerReachCommand->IsScheduled()) {
                        // ...and not scheduled, then start the command.
                        innerReachCommand->Schedule();
                        std::cout << "Scheduled inner reach command" << std::endl;
                    } else {
                        std::cout << "Command for inner reach already scheduled" << std::endl;
                    }
                } else {
                    std::cout << "No command for inner reach available.  Configure inner reach settings first." << std::endl;
                }
            }

            // Reset dashboard checkbox back to empty, so use knows they can click to activate again.
            frc::SmartDashboard::PutBoolean(DASH_INNER_REACH_ACTIVATE, false);
        });
    }

    {
        // Use dashboard to control inner arm rotation.

        // We'll create a new command as the user changes parameters on the dashboard (e.g. target position).
        static RotateInnerArmsCommand * innerRotateCommand = nullptr;

        static PID rotatePid { 0.0, 0.0, 0.0, 0.1, 0.5, -0.3, 0.3 };

        // Allow user to modify target position.
        static double innerRotateTarget = frc::SmartDashboard::GetNumber(DASH_INNER_ROTATE_TARGET, 0.0);
        updateDashboardNumber(DASH_INNER_ROTATE_TARGET, innerRotateTarget, [&](double target) {
            // Unsure what values constitute a safe range.  BE CAREFUL.

            if (nullptr != innerRotateCommand) {
                // Found an existing rotate command.
                if (innerRotateCommand->IsScheduled()) {
                    // Stop active rotate command.
                    innerRotateCommand->Cancel();
                    std::cout << "Canceled current inner rotate command" << std::endl;
                }
                // Delete object to free allocated memory to the OS.
                delete innerRotateCommand;
            }

            // Allocate a fresh command object.
            innerRotateCommand = new RotateInnerArmsCommand(mInnerRotate, target, rotatePid);
            std::cout << "Created new inner rotate command" << std::endl;

            // Update dashboard with new value, if limited.  Not necessary,
            // since target was clamped to a "safe" range.
            frc::SmartDashboard::PutNumber(DASH_INNER_ROTATE_TARGET, target);
        });

        // Allow user to start the command.
        static bool activateCommand = frc::SmartDashboard::GetBoolean(DASH_INNER_ROTATE_ACTIVATE, false);
        updateDashboardBool(DASH_INNER_ROTATE_ACTIVATE, activateCommand, [&](bool shallActivate) {
            bool isEnabled = frc::SmartDashboard::GetBoolean(DASH_USE_INNER_ROTATION_TEST_COMMAND, false);

            // Do nothing when flag is not active OR command is disabled.
            if (isEnabled && shallActivate) {
                if (nullptr != innerRotateCommand) {
                    // If a command is loaded...
                    if (! innerRotateCommand->IsScheduled()) {
                        // ...and not scheduled, then start the command.
                        innerRotateCommand->Schedule();
                        std::cout << "Scheduled inner rotate command" << std::endl;
                    } else {
                        std::cout << "Command for inner rotate already scheduled" << std::endl;
                    }
                } else {
                    std::cout << "No command for inner rotate available.  Configure inner rotate settings first." << std::endl;
                }
            } else if (shallActivate) {
                std::cout << "Command not enabled for inner rotate." << std::endl;
            }

            // Reset dashboard checkbox back to empty, so use knows they can click to activate again.
            frc::SmartDashboard::PutBoolean(DASH_INNER_ROTATE_ACTIVATE, false);
        });
    }

    {
        // Use dashboard to control swing.

        // We'll create a new command as the user changes parameters on the
        // dashboard (e.g. outerTargetPosition, innerTargetPosition, etc).
        static frc2::SequentialCommandGroup * swingCommand = nullptr;

        // Allow user to modify inner target position.
        static double innerReachTarget = frc::SmartDashboard::GetNumber(DASH_SWING_INNER_REACH_TARGET, 0.0);
        updateDashboardNumber(DASH_SWING_INNER_REACH_TARGET, innerReachTarget, [&](double target) {
            // Ensure value has a safe range.
            target = std::clamp(target, 0.5, 27.5);  // These limits try to protect from overshooting.  Max range [0, 28].

            if (nullptr != swingCommand) {
                if (swingCommand->IsScheduled()) {
                    swingCommand->Cancel();
                    std::cout << "Canceled current swing command" << std::endl;
                }
                delete swingCommand;
                swingCommand = nullptr;
            }

            // Many fields update this command.  To avoid duplicating code to
            // construct the command, Write the code after updating all field
            // variables.

            // Update dashboard with new value, if limited.
            frc::SmartDashboard::PutNumber(DASH_SWING_INNER_REACH_TARGET, target);
        });

        // Allow user to modify outer target position.
        static double outerReachTarget = frc::SmartDashboard::GetNumber(DASH_SWING_OUTER_REACH_TARGET, 0.0);
        updateDashboardNumber(DASH_SWING_OUTER_REACH_TARGET, outerReachTarget, [&](double target) {
            // Ensure value has a safe range.
            target = std::clamp(target, 0.5, 27.5);  // These limits try to protect from overshooting.  Max range [0, 28].

            if (nullptr != swingCommand) {
                if (swingCommand->IsScheduled()) {
                    swingCommand->Cancel();
                    std::cout << "Canceled current swing command" << std::endl;
                }
                delete swingCommand;
                swingCommand = nullptr;
            }

            // Many fields update this command.  To avoid duplicating code to
            // construct the command, Write the code after updating all field
            // variables.

            // Update dashboard with new value, if limited.
            frc::SmartDashboard::PutNumber(DASH_SWING_OUTER_REACH_TARGET, target);
        });

        // Create new command, if deleted.  But only once for this periodic invocation.
        if (nullptr == swingCommand) {
            swingCommand = new frc2::SequentialCommandGroup(
                frc2::InstantCommand { [=]() {
                    auto activeCmd = frc2::CommandScheduler::GetInstance().Requiring(mInnerReach);
                    if (nullptr != activeCmd) {
                        activeCmd->Cancel();
                    }
                }},
                frc2::InstantCommand { [=]() {
                    auto activeCmd = frc2::CommandScheduler::GetInstance().Requiring(mOuterReach);
                    if (nullptr != activeCmd) {
                        activeCmd->Cancel();
                    }
                }},
                frc2::ParallelCommandGroup {
                    ReachInnerArmsCommand { mInnerReach, innerReachTarget },
                    ReachOuterArmsCommand { mOuterReach, outerReachTarget }
                }
            );
        }

        // Allow user to start the command.
        static bool activateCommand = frc::SmartDashboard::GetBoolean(DASH_SWING_ACTIVATE, false);
        updateDashboardBool(DASH_SWING_ACTIVATE, activateCommand, [&](bool shallActivate) {
            bool isEnabled = frc::SmartDashboard::GetBoolean(DASH_SWING_USE_COMMAND, false);

            // Do nothing when flag is not active OR command is disabled.
            if (isEnabled && shallActivate) {
                if (nullptr != swingCommand) {
                    // If a command is loaded...
                    if (! swingCommand->IsScheduled()) {
                        // ...and not scheduled, then start the command.
                        swingCommand->Schedule();
                        std::cout << "Scheduled swing command" << std::endl;
                    } else {
                        std::cout << "Command for swing already scheduled" << std::endl;
                    }
                } else {
                    std::cout << "No command for swing available.  Configure swing settings first" << std::endl;
                }
            }

            // Reset dashboard checkbox back to empty, so user knows they can click to activate again.
            frc::SmartDashboard::PutBoolean(DASH_SWING_ACTIVATE, false);
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
