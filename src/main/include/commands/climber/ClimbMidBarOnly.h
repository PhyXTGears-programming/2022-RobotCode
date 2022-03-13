#pragma once

#include "cpptoml.h"

#include "climber/InnerReach.h"
#include "climber/InnerRotate.h"

#include <frc2/command/SequentialCommandGroup.h>

class ClimbMidBarOnly {
    public:
        ClimbMidBarOnly(InnerReach * innerArms, InnerRotate * innerRotate, std::shared_ptr<cpptoml::table> toml);

        
        frc2::SequentialCommandGroup * mReachMidBar = nullptr;
        frc2::SequentialCommandGroup * mClimbMidBarAndLock = nullptr;

    private:
        struct {
            double initialExtension;    // target distance to raise arms to clear mid bar.
            double liftRetraction;      // target distance to retract arms to lift robot on mid bar.
            double verticalArmAngle;    // angle when arms are vertical.
        } config;
};