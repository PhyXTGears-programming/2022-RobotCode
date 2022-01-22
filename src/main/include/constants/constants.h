#pragma once

#include "frc2/command/SubsystemBase.h"

namespace Constants
{

    //global constants

    //robot constraints for swerve in meters
    const double wheelBase = 20; //currently arbitrary
    const double trackWidth = 20; //currently arbitrary
    // pythagorean theorem of the wheelbase and track width to find hypotenuse
    const double diameter = sqrt(pow(wheelBase, 2) + pow(trackWidth, 2));


    const double MaxWheelSpeed = 20; //meters per sec

};