#pragma once

#include "frc2/command/SubsystemBase.h"

class Constants : public frc2::SubsystemBase
{
public:
    Constants(); // constructor

    //global constants

    //robot constraints for swerve
    double wheelBase = 20;
    double trackWidth = 20;
    // pythagorean theorem of the wheelbase and track width to find hypotenuse
    double diameter = sqrt(pow(wheelBase, 2) + pow(trackWidth, 2));


    
};