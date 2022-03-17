#pragma once

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "drivetrain-swerve/SwerveWheel.h"
#include "./swerve/drive.h"

#include <AHRS.h>

class SwerveDrive : public frc2::SubsystemBase {
    public:
        SwerveDrive(bool fieldOriented = false);

        void Periodic() override;

        void synchronizeTurnEncoders();

        void resetGyro();

        void enableFieldCentric();

        void disableFieldCentric();

        void setMotion(double x, double y, double r);

    private:
        swervedrive::drive<double, double, double>* drive;

        SwerveWheel flWheel {constants::swerve::frontLeft};
        SwerveWheel frWheel {constants::swerve::frontRight};
        SwerveWheel blWheel {constants::swerve::backLeft};
        SwerveWheel brWheel {constants::swerve::backRight};

        AHRS * gyro = nullptr;

        bool fieldOriented;
};
