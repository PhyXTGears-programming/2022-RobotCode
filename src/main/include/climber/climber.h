#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Servo.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class Climber : public frc2::SubsystemBase{
    public:
        Climber(std::shared_ptr<cpptoml::table> toml); // constructor

        void Periodic() override;

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
        void unlockArms();

        void rotateInner(double speed);
        void rotateOuter(double speed);

        void setInnerMotorsCoast();
        void setInnerMotorsBrake();

        void setRotateMotorsCoast();
        void setRotateMotorsBrake();

        double getInnerAngle();
        double getOuterAngle();

        void setInnerUnderLoad(bool isUnderLoad);
        void setOuterUnderLoad(bool isUnderLoad);

        double getInner1Position();
        double getInner2Position();
        double getOuter1Position();
        double getOuter2Position();

        bool isOuter1NearTarget(double target);
        bool isOuter2NearTarget(double target);
        bool isInner1NearTarget(double target);
        bool isInner2NearTarget(double target);

    private:
        bool mIsInnerUnderLoad = false;
        bool mIsOuterUnderLoad = false;

        rev::CANSparkMax mInnerHookMotor1 {interfaces::kInnerArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mInnerHookMotor2 {interfaces::kInnerArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mOuterHookMotor1 {interfaces::kOuterArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mOuterHookMotor2 {interfaces::kOuterArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        rev::CANSparkMax mOuterArmRotationMotor {interfaces::kOuterArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mInnerArmRotationMotor {interfaces::kInnerArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        frc::DutyCycleEncoder mInnerRotationEncoder {interfaces::kInnerRotationEncoder};
        frc::DutyCycleEncoder mOuterRotationEncoder {interfaces::kOuterRotationEncoder};

        rev::SparkMaxRelativeEncoder mInnerHook1Encoder {mInnerHookMotor1.GetEncoder()};
        rev::SparkMaxRelativeEncoder mInnerHook2Encoder {mInnerHookMotor2.GetEncoder()};
        rev::SparkMaxRelativeEncoder mOuterHook1Encoder {mOuterHookMotor1.GetEncoder()};
        rev::SparkMaxRelativeEncoder mOuterHook2Encoder {mOuterHookMotor2.GetEncoder()};

        frc::Servo mStopServo1 {interfaces::kStopServo1};
        frc::Servo mStopServo2 {interfaces::kStopServo2};

        struct {
            double extendSpeed, retractSpeed;

            struct {
                double unlockPosition;
                double lockPosition;
            } servo1, servo2;

            double inchesPerRevolution;
            struct {
                double innerStaticFriction, outerStaticFriction;
                double innerStaticFrictionWithLoad, outerStaticFrictionWithLoad;
            } friction;
        } config;
};