#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "constants/interfaces.h"

class Shooter : public frc2::SubsystemBase {
public:
    void runShooter (double speed);
    void stopShooter ();

private:
    rev::CANSparkMax mShooterMotor {interfaces::kShooterMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
            
};