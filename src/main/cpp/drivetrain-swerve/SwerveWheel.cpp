#include "drivetrain-swerve/SwerveWheel.h"

#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "drivetrain-swerve/swerve/rotation.h"

SwerveWheel::SwerveWheel (constants::swerve::WheelConstants constants)
    : swervedrive::swerve_module<double, double, double>(
        swervedrive::vector2<double>{constants.position.x, constants.position.y}
    ) {
    wheelSettings = constants;

    driveMotor = new rev::CANSparkMax(wheelSettings.drivePin, rev::CANSparkMax::MotorType::kBrushless);
    turnMotor = new rev::CANSparkMax(wheelSettings.turnPin, rev::CANSparkMax::MotorType::kBrushless);

    turnEncoder.emplace(turnMotor->GetEncoder());
    turnPid.emplace(turnMotor->GetPIDController());

    driveMotor->SetInverted(false);
    turnMotor->SetInverted(true);

    driveMotor->SetClosedLoopRampRate(0.6);
    turnMotor->SetClosedLoopRampRate(0.6);

    driveMotor->SetOpenLoopRampRate(0.6);
    driveMotor->SetOpenLoopRampRate(0.6);

    encoder = new ctre::phoenix::sensors::CANCoder(wheelSettings.encoderID);
    encoder->ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);

    constexpr double conversionFactor = 2.0 * M_PI / 56.0; // motor rotations to module rad
    turnEncoder->SetPositionConversionFactor(conversionFactor);
    turnEncoder->SetVelocityConversionFactor(conversionFactor / 60.0); // RPM to rad/s

    double currentAngle = encoder->GetAbsolutePosition() * (M_PI / 180.0); // [-180, 180) to [-pi, pi)
    turnEncoder->SetPosition(currentAngle - wheelSettings.tuning.zeroVal);

    turnPid->SetP(wheelSettings.tuning.pid.P);
    turnPid->SetI(wheelSettings.tuning.pid.I);
    turnPid->SetD(wheelSettings.tuning.pid.D);
}

void SwerveWheel::drive (double speed, double angle) {
    if (std::fabs(speed) > 0.05) {  // (jcc Mar02) Another attempt to prevent wild steering near deadzone.
        setAngle(angle);
    }
    setSpeed(speed);
}

double SwerveWheel::getAngle () {
    return turnEncoder->GetPosition();
}

void SwerveWheel::setAngle (double rad) {
    double correction = std::copysign(0.04, turnEncoder->GetVelocity()); // add correction to account for lag and momentum

    std::pair<double, bool> result = swervedrive::module_rotation::calculate_rotation_target(rad, getAngle() + correction);
    inverted = result.second;

    turnPid->SetReference(result.first, rev::CANSparkMax::ControlType::kPosition);
}
