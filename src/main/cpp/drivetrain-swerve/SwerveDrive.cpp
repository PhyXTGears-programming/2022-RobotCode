#include "drivetrain-swerve/SwerveDrive.h"

#include <frc/smartdashboard/SmartDashboard.h>

constexpr double PI = 3.1415926535897932;

SwerveDrive::SwerveDrive(bool fieldOriented) : fieldOriented(fieldOriented)
{
    drive = new swervedrive::drive<double, double, double>({&flWheel, &frWheel, &blWheel, &brWheel});

    gyro = new AHRS(frc::SPI::kMXP);
    resetGyro();
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

    frc::SmartDashboard::PutNumber("Gyro Angle", getHeading() * 180.0 / M_PI);

    frc::SmartDashboard::PutBoolean("Field Oriented Steering On", fieldOriented);
}

void SwerveDrive::synchronizeTurnEncoders()
{
    flWheel.synchronizeTurnEncoder();
    frWheel.synchronizeTurnEncoder();
    blWheel.synchronizeTurnEncoder();
    brWheel.synchronizeTurnEncoder();
}

void SwerveDrive::resetGyro(){
    gyro->Reset();
}

void SwerveDrive::enableFieldCentric(){
    fieldOriented = true;
}

void SwerveDrive::disableFieldCentric(){
    fieldOriented = false;
}

void SwerveDrive::toggleFieldCentric() {
    fieldOriented = !fieldOriented;
}

void SwerveDrive::setMotion(double x, double y, double r)
{
    double a = 0;

    if (fieldOriented) {
        // Swerve drive expects heading angle to become more positive with
        // counter-clockwise (CCW) rotation, and more negative with clockwise
        // (CW) rotation.
        // NavX angle is (+) on CW, and (-) on CCW.  Use negate to fix heading
        // for swerve drive.
        a = -getHeading();
    }

    drive->set_motion({x, y}, r, a);
}

double SwerveDrive::getHeading(){
    return gyro->GetAngle() * (PI/180.0);
}