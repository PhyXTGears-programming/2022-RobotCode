#include "commands/calibrate/CalibrateWheelOffsetsCommand.h"

#include <frc2/command/CommandState.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include <functional>
#include <iostream>

CalibrateWheelOffsetsCommand::CalibrateWheelOffsetsCommand (
    SwerveDrive* swerve,
    std::function<bool ()> isCalibrationDone,
    std::function<bool ()> isDriveForwardActive,
) {
    AddRequirements(swerve);
    mSwerve = swerve;

    mProcedure = new frc2::SequentialCommandGroup {
        // Reset offsets for absolute encoders to zero.
        frc2::InstantCommand {
            [=]() { mSwerve->calibrateClearZeroOffsets(); },
            { mSwerve }
        },

        // Move wheels to zero.
        frc2::InstantCommand {
            [=]() { mSwerve->calibrateOrientWheels(0.0); },
            { mSwerve }
        },

        // Disable wheel motor controllers, so humans can rotate wheels to point forward.
        frc2::InstantCommand {
            [=]() { mSwerve->calibrateDisableMotors(); },
            { mSwerve }
        },

        // Wait for humans to continue, but allow them to drive wheels to verify wheels aren't pointed backwards.
        frc2::FunctionalCommand {
            /* Initialize */ []() {

            },
            /* Execute */ [=]() {
                if (isDriveForwardActive()) {
                    mSwerve->calibrateDriveFoward();
                } else {
                    mSwerve->calibrateDisableMotors();
                }
            },
            /* End */ [=](bool interrupted) {
                mSwerve->calibrateDisableMotors();
            },
            /* IsFinished */ [=]() {
                return isCalibrationDone();
            },
            { mSwerve }
        },
        
        // When humans are done...

        // Update offsets in absolute encoders.
        frc2::InstantCommand {
            [=]() {
                WheelOffsets offsets = mSwerve->calibrateGetZeroOffsets();

                std::cout << "Wheel offset calibrations results:" << std::endl
                    << "   front left zero : " << offsets.frontLeft << std::endl
                    << "   front right zero: " << offsets.frontRight << std::endl
                    << "   back left zero  : " << offsets.backLeft << std::endl
                    << "   back right zero : " << offsets.backRight << std::endl
                    << std::endl;

                mSwerve->calibrateSetZeroOffsets(offsets);

                mWheelOffsets = { offsets };
            },
            { mSwerve }
        },

        // Write offsets to disk.
        frc2::InstantCommand {
            [=]() {},
            {}
        },
    };
}

void CalibrateWheelOffsetsCommand::Initialize () {
    mProcedure->Initialize();
}

void CalibrateWheelOffsetsCommand::Execute () {
    mProcedure->Execute();
}

void CalibrateWheelOffsetsCommand::End (bool interrupted) {
    mProcedure->End(interrupted);
}

bool CalibrateWheelOffsetsCommand::IsFinished () {
    return mProcedure->IsFinished();
}