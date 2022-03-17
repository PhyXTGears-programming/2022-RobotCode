#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class ClimberOuterRotate : public frc2::SubsystemBase {
    public:
        ClimberOuterRotate(std::shared_ptr<cpptoml::table> toml);

        void Periodic();

        void rotate(double speed);
        void stop();
    
        double getAngle();

        void setMotorCoast();
        void setMotorBrake();

        void setCurrentlimit(unsigned int limit);
        void resetCurrentLimit();

    private:
        rev::CANSparkMax mMotor {interfaces::kOuterArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::SparkMaxRelativeEncoder mMotorEncoder = mMotor.GetEncoder();

        frc::DutyCycleEncoder mEncoder {interfaces::kOuterRotationEncoder};
};