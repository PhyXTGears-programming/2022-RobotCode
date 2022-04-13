#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "cpptoml.h"
#include "constants/interfaces.h"
#include "frc/Solenoid.h"
#include "frc/PneumaticsModuleType.h"

class Intake : public frc2::SubsystemBase {
public:
    Intake (std::shared_ptr<cpptoml::table> toml);

    void runRollers ();
    void runRollersReverse ();
    void stopRollers ();

    void extend ();
    void retract ();

    bool isExtended ();

private:
    rev::CANSparkMax mRollerMotor {interfaces::kRollerMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; // the motor that turns the rollers to pull the ball in
    
    frc::Solenoid mExtendLeft {frc::PneumaticsModuleType::CTREPCM, interfaces::kIntakeExtendLeft};
    frc::Solenoid mRetractLeft {frc::PneumaticsModuleType::CTREPCM, interfaces::kIntakeRetractLeft};
    frc::Solenoid mExtendRight {frc::PneumaticsModuleType::CTREPCM, interfaces::kIntakeExtendRight};
    frc::Solenoid mRetractRight {frc::PneumaticsModuleType::CTREPCM, interfaces::kIntakeRetractRight};

    bool mIsExtended = false;

    struct {
        double rollerSpeed;
    } config;
};