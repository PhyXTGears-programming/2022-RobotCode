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

void SwerveDrive::calibrateClearZeroOffsets() {
    flWheel.clearZeroOffset();
    frWheel.clearZeroOffset();
    blWheel.clearZeroOffset();
    brWheel.clearZeroOffset();
}

void SwerveDrive::calibrateOrientWheels(double radians) {
    flWheel.setAngle(radians);
    frWheel.setAngle(radians);
    blWheel.setAngle(radians);
    brWheel.setAngle(radians);
}

void SwerveDrive::calibrateDisableMotors() {
    flWheel.disableMotors();
    frWheel.disableMotors();
    blWheel.disableMotors();
    brWheel.disableMotors();
}

void SwerveDrive::calibrateDriveFoward() {
    flWheel.setSpeed(0.2);
    frWheel.setSpeed(0.2);
    blWheel.setSpeed(0.2);
    brWheel.setSpeed(0.2);
}

WheelOffsets SwerveDrive::calibrateGetZeroOffsets() {
    WheelOffsets ret;

    ret.frontLeft = flWheel.getAbsAngle();
    ret.frontRight = frWheel.getAbsAngle();
    ret.backLeft = blWheel.getAbsAngle();
    ret.backRight = brWheel.getAbsAngle();

    return ret;
}

void SwerveDrive::calibrateSetZeroOffsets(WheelOffsets & const offsets) {
    flWheel.setZeroOffset(offsets.frontLeft);
    frWheel.setZeroOffset(offsets.frontRight);
    blWheel.setZeroOffset(offsets.backLeft);
    brWheel.setZeroOffset(offsets.backRight);
}

void SwerveDrive::resetGyro() {
    gyro->Reset();
}

void SwerveDrive::enableFieldCentric() { fieldOriented = true; }

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