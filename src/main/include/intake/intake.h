#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/AnalogPotentiometer.h>

class Intake : public frc2::SubsystemBase {
public:

private:
    TalonSRX mRollerMotor {0}; // the motor that turns the rollers to pull the ball in
    TalonSRX mDeployMotor {0}; // the motor that flips the intake in/out

    frc::AnalogPotentiometer mIntakePosition {0, 360.0, 0.0}; // {channel, full range, offset}
};