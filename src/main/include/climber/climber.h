#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include "constants/interfaces.h"
#include <cpptoml.h>

class Climber : public frc2::SubsystemBase{
    public:
        Climber(std::shared_ptr<cpptoml::table> toml); // constructor

        void extendOuter();
        void retractOuter();
        void extendInner();
        void retractInner();
        void lockArms();
        void updateRelay();

        double getInnerArmRotations();
        double getOuterArmRotations();

    private:
        const int kCountsPerRevolution = 42;

        rev::CANSparkMax mInnerHookMotor1 {interfaces::kInnerArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mInnerHookMotor2 {interfaces::kInnerArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mOuterHookMotor1 {interfaces::kOuterArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mOuterHookMotor2 {interfaces::kOuterArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        rev::SparkMaxRelativeEncoder mInnerHook1Encoder = mInnerHookMotor1.getEncoder();
        rev::SparkMaxRelativeEncoder mInnerHook2Encoder = mInnerHookMotor2.getEncoder();
        rev::SparkMaxRelativeEncoder mOuterHook1Encoder = mOuterHookMotor1.getEncoder();
        rev::SparkMaxRelativeEncoder mOuterHook2Encoder = mOuterHookMotor2.getEncoder();

        frc::Servo mStopServo1 {interfaces::kStopServo1};
        frc::Servo mStopServo2 {interfaces::kStopServo2};
        frc::Servo mInnerServo1 {interfaces::kInnerServo1};
        frc::Servo mInnerServo2 {interfaces::kInnerServo2};
        frc::Servo mOuterServo1 {interfaces::kOuterServo1};
        frc::Servo mOuterServo2 {interfaces::kOuterServo2};

        frc::Relay mBackDriveRelay {interfaces::kBackDriveRelay};

        struct {
            double extendSpeed, retractSpeed;
            double lockServoPosition;
            struct {
                double innerStaticFriction, outerStaticFriction;
                double innerStaticFrictionWithLoad, outerStaticFrictionWithLoad;
            } friction;
        } config;
};