#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>

#include "constants/interfaces.h"
#include "cpptoml.h"

class Shooter : public frc2::SubsystemBase {
public:
    Shooter (std::shared_ptr<cpptoml::table> toml);
    
    void runShooter (double speed);
    void shootFar ();
    void shootNear ();
    void stopShooter ();

private:
    rev::CANSparkMax mShooterMotor {interfaces::kShooterMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

    /**
     * @brief Set the PID Values of the motor
     * 
     * @param PIDController the motor controller PID controller object
     * @param k_P the P value to set it to
     * @param k_I the I value to set it to
     * @param k_D the D value to set it to
     * @param k_FF the Feed foreward value to set it to
     * @param k_minValue the minimum output value of the PID loop
     * @param k_maxValue the miximum output value of the PID loop
     * @param k_IZone the I zone value to set it to (defaults to 0)
     */
    void setPidValues(rev::SparkMaxPIDController PIDController, double k_P, double k_I, double k_D, double k_FF, double k_minValue, double k_maxValue, double k_IZone = 0.0);

    rev::SparkMaxPIDController mPID_ShooterMotor = Shooter::mShooterMotor.GetPIDController();          

    struct {
        struct {
            double near;
            double far;
        } speed;

        struct {
            double p;
            double i;
            double d;
            double izone;
            double ff;
            double minValue;
            double maxValue;
        } motor;
    } config;
};