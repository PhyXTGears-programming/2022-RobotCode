#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/AnalogPotentiometer.h>

class Intake : public frc2::SubsystemBase {
public:
    void extendIntake ();
    void retractIntake ();

    void moveIntake ();

    void runRollers (double speed);
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

    const double kExtendTargetSpeed = 0.5;
    const double kRetractTargetSpeed = -0.5;

    double mIntakeRetractedPosition = 0.0; // value the potentiometer should read when retracted
    double mIntakeExtendedPosition = 90.0; // value the potentiometer should read when extended
};