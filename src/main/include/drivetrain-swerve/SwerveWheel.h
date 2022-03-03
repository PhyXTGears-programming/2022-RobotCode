#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>

#include "./Constants.h"
#include "./swerve/vector2.h"
#include "./swerve/swerve_module.h"

class SwerveWheel : public swervedrive::swerve_module<double, double, double> {
    public:
        SwerveWheel(constants::swerve::WheelConstants constants);

        void drive(double speed, double angle) override;

        double getAngle();

    private:
        void setAngle(double angle);
        void setSpeed (double speed) {
            driveMotor->Set(inverted ? -speed : speed);
        }

        constants::swerve::WheelConstants wheelSettings;
        
        rev::CANSparkMax *driveMotor, *turnMotor;
        std::optional<rev::SparkMaxRelativeEncoder> turnEncoder;
        std::optional<rev::SparkMaxPIDController> turnPid;
        ctre::phoenix::sensors::CANCoder* encoder;

        bool inverted = false;

        std::string labelError;
};
