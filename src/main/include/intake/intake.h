#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Servo.h>
#include <rev/CANSparkMax.h>
#include <frc/AnalogPotentiometer.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class Intake : public frc2::SubsystemBase {
public:
    Intake (std::shared_ptr<cpptoml::table> toml);

    void extendIntake ();
    void retractIntake ();

    void runRollers ();
    void stopRollers ();

    bool isIntakeExtended ();

    void setStationary (bool isExtended); // sets mCurrentIntakeStatus to stationary

private:
    enum mIntakeMovementStatus {EXTENDING, RETRACTING, STATIONARY};
    mIntakeMovementStatus mCurrentIntakeStatus = STATIONARY;

    rev::CANSparkMax mRollerMotor {interfaces::kRollerMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; // the motor that turns the rollers to pull the ball in

    frc::Servo mDeployServo1 {interfaces::kDeployServo1};
    frc::Servo mDeployServo2 {interfaces::kDeployServo2};

    bool mIsIntakeExtended = false;

    struct {
        double rollerSpeed, extendTargetSpeed, retractTargetSpeed;
        double intakeRetractedPosition, intakeExtendedPosition;
    } config;
    
};