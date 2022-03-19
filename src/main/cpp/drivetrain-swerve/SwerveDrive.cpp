#include "drivetrain-swerve/SwerveDrive.h"

#include <frc/smartdashboard/SmartDashboard.h>

constexpr double PI = 3.1415926535897932;

SwerveDrive::SwerveDrive(bool fieldOriented) : fieldOriented(fieldOriented)
{
    drive = new swervedrive::drive<double, double, double>({&flWheel, &frWheel, &blWheel, &brWheel});

    // gyro.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
    // gyro.Reset();
}

void SwerveDrive::Periodic()
{
    frc::SmartDashboard::PutNumber("Steer FR Rel", frWheel.getAngle() / M_PI * 180);
    frc::SmartDashboard::PutNumber("Steer FL Rel", flWheel.getAngle() / M_PI * 180);
    frc::SmartDashboard::PutNumber("Steer BR Rel", brWheel.getAngle() / M_PI * 180);
    frc::SmartDashboard::PutNumber("Steer BL Rel", blWheel.getAngle() / M_PI * 180);

    frc::SmartDashboard::PutNumber("Steer FR Abs", frWheel.getAbsAngle() / M_PI * 180);
    frc::SmartDashboard::PutNumber("Steer FL Abs", flWheel.getAbsAngle() / M_PI * 180);
    frc::SmartDashboard::PutNumber("Steer BR Abs", brWheel.getAbsAngle() / M_PI * 180);
    frc::SmartDashboard::PutNumber("Steer BL Abs", blWheel.getAbsAngle() / M_PI * 180);
}

void SwerveDrive::synchronizeTurnEncoders()
{
    flWheel.synchronizeTurnEncoder();
    frWheel.synchronizeTurnEncoder();
    blWheel.synchronizeTurnEncoder();
    brWheel.synchronizeTurnEncoder();
}

void SwerveDrive::setMotion(double x, double y, double r)
{
    double a = 0;

    if (fieldOriented)
    {
        // a = gyro.GetAngle() * (PI/180.0);
    }

    drive->set_motion({x, y}, r, a);
}