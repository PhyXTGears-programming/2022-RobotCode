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

        class InnerReach : public frc2::SubsystemBase {
            InnerReach();

            void extend1();
            void extend2();
            void retract1();
            void retract2();
            void stop1();
            void stop2();
            void run1(double speed);
            void run2(double speed);

            void setMotorsCoast();
            void setMotorsBrake();

            void lockArms();
            void unlockArms();

            double getMotor1Position();
            double getMotor2Position();

            bool isMotor1NearTarget(double target);
            bool isMotor2NearTarget(double target);

            void setUnderLoad(bool isUnderLoad);

            bool mIsUnderLoad = false;

            rev::CANSparkMax mMotor1 {interfaces::kInnerArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
            rev::CANSparkMax mMotor2 {interfaces::kInnerArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

            rev::SparkMaxRelativeEncoder mEncoder1 {mMotor1.GetEncoder()};
            rev::SparkMaxRelativeEncoder mEncoder2 {mMotor2.GetEncoder()};

            frc::Servo mStopServo1 {interfaces::kStopServo1};
            frc::Servo mStopServo2 {interfaces::kStopServo2};
        };

        class OuterReach : public frc2::SubsystemBase {
            OuterReach();

            void extend1();
            void extend2();
            void retract1();
            void retract2();
            void stop1();
            void stop2();
            void run1(double speed);
            void run2(double speed);

            void setMotorsCoast();
            void setMotorsBrake();

            double getMotor1Position();
            double getMotor2Position();

            bool isMotor1NearTarget(double target);
            bool isMotor2NearTarget(double target);

            void setUnderLoad(bool isUnderLoad);

            bool mIsUnderLoad = false;

            rev::CANSparkMax mMotor1 {interfaces::kOuterArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
            rev::CANSparkMax mMotor2 {interfaces::kOuterArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

            rev::SparkMaxRelativeEncoder mEncoder1 {mMotor1.GetEncoder()};
            rev::SparkMaxRelativeEncoder mEncoder2 {mMotor2.GetEncoder()};
        };

        class InnerRotate : public frc2::SubsystemBase {
            InnerRotate();

            void rotate(double speed);
            
            double getAngle();

            void setMotorCoast();
            void setMotorBrake();

            rev::CANSparkMax mMotor {interfaces::kInnerArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

            frc::DutyCycleEncoder mEncoder {interfaces::kInnerRotationEncoder};
        };

        class OuterRotate : public frc2::SubsystemBase {
            public:
                OuterRotate();

                void rotate(double speed);
            
                double getAngle();

                void setMotorCoast();
                void setMotorBrake();
            
            private:
                rev::CANSparkMax mMotor {interfaces::kOuterArmRotation, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

                frc::DutyCycleEncoder mEncoder {interfaces::kOuterRotationEncoder};
        };

        static struct {
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

        static const double kAcceptablePositionError = 0.3;
};