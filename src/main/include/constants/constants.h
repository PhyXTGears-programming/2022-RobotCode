#pragma once

#include "frc2/command/SubsystemBase.h"

#define FEET_TO_METERS(feet) (feet * 0.3048)


namespace constants
{

    //global constants

    //robot constraints for swerve in meters
    const double kWheelBase = FEET_TO_METERS(20/12); //20 in wheelbase square
    const double kTrackWidth = FEET_TO_METERS(20/12);
    // pythagorean theorem of the wheelbase and track width to find hypotenuse
    const double kDiameter = sqrt(pow(kWheelBase, 2) + pow(kTrackWidth, 2));


    const double kMaxWheelSpeed = 4.5; //meters per sec

    namespace climb {
        const double kAcceptableAngleError = 0.5;
        const double kAcceptablePositionError = 0.5;
        const double kAcceptableVelocityError = 0.75;
    }
}