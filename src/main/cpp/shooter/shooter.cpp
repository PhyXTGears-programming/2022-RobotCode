#include "shooter/shooter.h"

#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter(std::shared_ptr<cpptoml::table> toml) {
    config.speed.near = toml->get_qualified_as<double>("speed.near").value_or(2700);
    config.speed.far = toml->get_qualified_as<double>("speed.far").value_or(3200);
    config.speed.lowHub = toml->get_qualified_as<double>("speed.lowHub").value_or(2000);
    config.speed.reverse = toml->get_qualified_as<double>("speed.lowHub").value_or(-1000);
    config.speed.auton = toml->get_qualified_as<double>("speed.auto").value_or(2500);
    config.motor.p = toml->get_qualified_as<double>("motor.p").value_or(0.01);
    config.motor.i = toml->get_qualified_as<double>("motor.i").value_or(0.0);
    config.motor.d = toml->get_qualified_as<double>("motor.d").value_or(0.0);
    config.motor.ff = toml->get_qualified_as<double>("motor.ff").value_or(0.0);
    config.motor.minValue = toml->get_qualified_as<double>("motor.minValue").value_or(-1.0);
    config.motor.maxValue = toml->get_qualified_as<double>("motor.maxValue").value_or(1.0);
    config.motor.izone = toml->get_qualified_as<double>("motor.izone").value_or(1.0);

    Shooter::setPidValues(
        mPID_ShooterMotor, 
        config.motor.p, 
        config.motor.i, 
        config.motor.d, 
        config.motor.ff, 
        config.motor.minValue, 
        config.motor.maxValue, 
        config.motor.izone
    );
}

void Shooter::Periodic () {
    // frc::SmartDashboard::PutNumber("Shooter Speed (rpm)", mShooterEncoder.GetVelocity());
    // frc::SmartDashboard::PutNumber("Shooter Speed (rpm) Graph", mShooterEncoder.GetVelocity());
}

void Shooter::runShooter (double speed) {
    mPID_ShooterMotor.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
}

void Shooter::shootLowHub() {
    runShooter(config.speed.lowHub);
}


void Shooter::shootReverse() {
    runShooter(config.speed.reverse);
}

void Shooter::shootFar() {
    runShooter(config.speed.far);
}

void Shooter::shootNear() {
    runShooter(config.speed.near);
}

void Shooter::shootAuto() {
    runShooter(config.speed.auton);
}

void Shooter::stopShooter () {
    mShooterMotor.StopMotor();
}

void Shooter::setPidValues(rev::SparkMaxPIDController PIDController, double k_P,
                            double k_I, double k_D, double k_FF,
                            double k_minValue, double k_maxValue, double k_IZone /*= 0.0*/)
{
    PIDController.SetP(k_P);
    PIDController.SetI(k_I);
    PIDController.SetD(k_D);
    PIDController.SetFF(k_FF);
    PIDController.SetOutputRange(k_minValue, k_maxValue);
    PIDController.SetIZone(k_IZone);
}
