#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class ClimberInnerRotate : public frc2::SubsystemBase {
    public:
        ClimberInnerRotate(std::shared_ptr<cpptoml::table> toml);

        void Periodic();

        void rotate(double speed);
        void stop();
        
        double getAngle();

        void setMotorCoast();
        void setMotorBrake();

        void setCurrentlimit(unsigned int limit);
        void resetCurrentLimit();

    private:
        rev::CANSparkMax mMotor {interfaces::kInnerArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::SparkMaxRelativeEncoder mMotorEncoder = mMotor.GetEncoder();

        frc::DutyCycleEncoder mEncoder {interfaces::kInnerRotationEncoder};
};