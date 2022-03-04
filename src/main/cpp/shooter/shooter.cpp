#include "shooter/shooter.h"

Shooter::Shooter(std::shared_ptr<cpptoml::table> toml) {
    config.speed.near = toml->get_qualified_as<double>("speed.near").value_or(0.1);
    config.speed.far = toml->get_qualified_as<double>("speed.far").value_or(0.5);
    config.motor.p = toml->get_qualified_as<double>("motor.p").value_or(0.001);
    config.motor.i = toml->get_qualified_as<double>("motor.i").value_or(0.0);
    config.motor.d = toml->get_qualified_as<double>("motor.d").value_or(0.0);
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

void Shooter::runShooter (double speed) {
    mPID_ShooterMotor.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
}

void Shooter::shootFar() {
    runShooter(config.speed.far);
}

void Shooter::shootNear() {
    runShooter(config.speed.near);
}

void Shooter::stopShooter () {
    runShooter(0.0);
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
