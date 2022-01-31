#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/AnalogPotentiometer.h>

class Intake : public frc2::SubsystemBase {
public:
    void extendIntake ();
    void retractIntake ();

    void moveIntake ();

    void runRollers ();
    void stopRollers ();

    bool isIntakeExtended ();

    double getIntakePosition ();

    double getDistanceToPosition ();

    void setStationary (bool isExtended); // sets mCurrentIntakeStatus to stationary

private:
    enum mIntakeMovementStatus {EXTENDING, RETRACTING, STATIONARY};
    mIntakeMovementStatus mCurrentIntakeStatus = STATIONARY;

    TalonSRX mRollerMotor {0}; // the motor that turns the rollers to pull the ball in
    TalonSRX mDeployMotor {0}; // the motor that flips the intake in/out

    frc::AnalogPotentiometer mIntakePosition {0, 360.0, 0.0}; // {channel, full range, offset}

    bool mIsIntakeExtended = false;

    double mDeployTargetSpeed = 1.0;
    double mDeployCurrentSpeed = 0.0;

    struct {
        double rollerSpeed, extendTargetSpeed, retractTargetSpeed;
        double deploySpeedFactor; // the larger this value is, the faster the deploy motor will try to accelerate
        double intakeRetractedPosition, intakeExtendedPosition; // the potentiometer readings when intake is fully extended/retracted
    } config;
    
};