#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Servo.h>

#include "cpptoml.h"
#include "constants/interfaces.h"
#include "constants/constants.h"

class ClimberInnerReach : public frc2::SubsystemBase {
    public:
        ClimberInnerReach(std::shared_ptr<cpptoml::table> toml);

        void Periodic() override;

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

        // void lockArms();
        // void unlockArms();

        double getMotor1Position();
        double getMotor2Position();

        bool isMotor1NearTarget(double target);
        bool isMotor2NearTarget(double target);

        void setUnderLoad(bool isUnderLoad);
    
    private:
        bool mIsUnderLoad = false;

        rev::CANSparkMax mMotor1 {interfaces::kInnerArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mMotor2 {interfaces::kInnerArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        rev::SparkMaxRelativeEncoder mEncoder1 {mMotor1.GetEncoder()};
        rev::SparkMaxRelativeEncoder mEncoder2 {mMotor2.GetEncoder()};

        // frc::Servo mStopServo1 {interfaces::kStopServo1};
        // frc::Servo mStopServo2 {interfaces::kStopServo2};

        struct {
            double extendSpeed, retractSpeed;

            // struct {
            //     double unlockPosition;
            //     double lockPosition;
            // } servo1, servo2;

            double inchesPerRevolution;
            struct {
                double innerStaticFriction;
                double innerStaticFrictionWithLoad;
            } friction;
        } config;
};