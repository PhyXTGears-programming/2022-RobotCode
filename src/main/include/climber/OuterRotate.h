#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class OuterRotate : public frc2::SubsystemBase {
    public:
        OuterRotate(std::shared_ptr<cpptoml::table> toml);

        void rotate(double speed);
    
        double getAngle();

        void setMotorCoast();
        void setMotorBrake();
    
    private:
        rev::CANSparkMax mMotor {interfaces::kOuterArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        frc::DutyCycleEncoder mEncoder {interfaces::kOuterRotationEncoder};
};