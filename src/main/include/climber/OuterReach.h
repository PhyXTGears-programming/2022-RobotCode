#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "cpptoml.h"
#include "constants/interfaces.h"

class OuterReach : public frc2::SubsystemBase {
    public:
        OuterReach(std::shared_ptr<cpptoml::table> toml);

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

    private:
        bool mIsUnderLoad = false;

        rev::CANSparkMax mMotor1 {interfaces::kOuterArm1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax mMotor2 {interfaces::kOuterArm2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        rev::SparkMaxRelativeEncoder mEncoder1 {mMotor1.GetEncoder()};
        rev::SparkMaxRelativeEncoder mEncoder2 {mMotor2.GetEncoder()};

        struct {
            double extendSpeed, retractSpeed;

            double inchesPerRevolution;
            struct {
                double innerStaticFriction, outerStaticFriction;
                double innerStaticFrictionWithLoad, outerStaticFrictionWithLoad;
            } friction;
        } config;
};