#pragma once

#include <string>

// #define TALON_SRX
#define SPARK_MAX

namespace constants {
    struct Vector {
        double x, y;
    };

    struct PIDTuning {
        double P, I, D;
    };

    namespace swerve {
        struct WheelSpinTuning {
            PIDTuning pid;
            double zeroVal; // value encoder reads when pointing forward (rad)
        };

        struct WheelConstants {
            std::string name;
            int drivePin; // PWM (talon) / CAN (spark max)
            int turnPin; // CAN
            Vector position;
            WheelSpinTuning tuning;
            int encoderID;
        };

        const WheelConstants frontLeft  = {"Front Left",  4, 8, {-0.476,  0.476}, {{0.7, 0, 0},  ( -4.0 )}, 13};
        const WheelConstants frontRight = {"Front Right", 1, 5, { 0.476,  0.476}, {{0.7, 0, 0},  (-161.36 - 90)}, 10};
        const WheelConstants backLeft   = {"Back Left",   3, 7, {-0.476, -0.476}, {{0.7, 0, 0},  (  63.90 + 90)}, 12};
        const WheelConstants backRight  = {"Back Right",  2, 6, { 0.476, -0.476}, {{0.7, 0, 0},  ( -4.0 + 180.0 )}, 11};
    }
}
