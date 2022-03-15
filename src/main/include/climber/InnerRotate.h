#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class ClimberInnerRotate : public frc2::SubsystemBase {
    public:
        ClimberInnerRotate(std::shared_ptr<cpptoml::table> toml);

        void rotate(double speed);
        void stop();
        
        double getAngle();

        void setMotorCoast();
        void setMotorBrake();

    private:
        rev::CANSparkMax mMotor {interfaces::kInnerArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        frc::DutyCycleEncoder mEncoder {interfaces::kInnerRotationEncoder};
};