#include "drivetrain-swerve/SwerveDrive.h"

#include <frc/smartdashboard/SmartDashboard.h>

constexpr double PI = 3.1415926535897932;

SwerveDrive::SwerveDrive (bool fieldOriented) : fieldOriented(fieldOriented) {
    drive = new swervedrive::drive<double, double, double>({&flWheel, &frWheel, &blWheel, &brWheel});

    // gyro.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
    // gyro.Reset();
}

void SwerveDrive::Periodic () {}

void SwerveDrive::synchronizeTurnEncoders () {
    flWheel.synchronizeTurnEncoder();
    frWheel.synchronizeTurnEncoder();
    blWheel.synchronizeTurnEncoder();
    brWheel.synchronizeTurnEncoder();
}

void SwerveDrive::setMotion (double x, double y, double r) {
    double a = 0;

    if (fieldOriented) {
        // a = gyro.GetAngle() * (PI/180.0);
    }

    drive->set_motion({x, y}, r, a);
}
