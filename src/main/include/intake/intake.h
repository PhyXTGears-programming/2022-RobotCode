#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class Intake : public frc2::SubsystemBase {
public:
    Intake (std::shared_ptr<cpptoml::table> toml);

    void runRollers ();
    void stopRollers ();

private:
    rev::CANSparkMax mRollerMotor {interfaces::kRollerMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; // the motor that turns the rollers to pull the ball in

    struct {
        double rollerSpeed;
    } config;
};