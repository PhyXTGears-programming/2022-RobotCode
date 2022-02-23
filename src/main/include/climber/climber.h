#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Servo.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class Climber : public frc2::SubsystemBase{
    public:
        Climber(std::shared_ptr<cpptoml::table> toml); // constructor

        void extendOuter1();
        void extendOuter2();
        void retractOuter1();
        void retractOuter2();
        void stopOuter1();
        void stopOuter2();

        void extendInner1();
        void extendInner2();
        void retractInner1();
        void retractInner2();
        void stopInner1();
        void stopInner2();

        void lockArms();

        void rotateInner(double speed);
        void rotateOuter(double speed);

        double getInnerAngle();
        double getOuterAngle();

        void setUnderLoad(bool isUnderLoad);

        bool isOuter1NearTarget(double target);
        bool isOuter2NearTarget(double target);
        bool isInner1NearTarget(double target);
        bool isInner2NearTarget(double target);

    private:
        bool mIsUnderLoad = false;

        rev::CANSparkMax mInnerHookMotor1 {interfaces::kInnerArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mInnerHookMotor2 {interfaces::kInnerArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mOuterHookMotor1 {interfaces::kOuterArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mOuterHookMotor2 {interfaces::kOuterArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        rev::CANSparkMax mOuterArmRotationMotor {interfaces::kOuterArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mInnerArmRotationMotor {interfaces::kInnerArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        ctre::phoenix::sensors::CANCoder mInnerRotationEncoder {interfaces::kInnerRotationEncoder};
        ctre::phoenix::sensors::CANCoder mOuterRotationEncoder {interfaces::kOuterRotationEncoder};

        rev::SparkMaxRelativeEncoder mInnerHook1Encoder {mInnerHookMotor1.GetEncoder()};
        rev::SparkMaxRelativeEncoder mInnerHook2Encoder {mInnerHookMotor2.GetEncoder()};
        rev::SparkMaxRelativeEncoder mOuterHook1Encoder {mOuterHookMotor1.GetEncoder()};
        rev::SparkMaxRelativeEncoder mOuterHook2Encoder {mOuterHookMotor2.GetEncoder()};

        frc::Servo mStopServo1 {interfaces::kStopServo1};
        frc::Servo mStopServo2 {interfaces::kStopServo2};

        struct {
            double extendSpeed, retractSpeed;
            double lockServoPosition;
            double inchesPerRevolution;
            struct {
                double innerStaticFriction, outerStaticFriction;
                double innerStaticFrictionWithLoad, outerStaticFrictionWithLoad;
            } friction;
        } config;
};