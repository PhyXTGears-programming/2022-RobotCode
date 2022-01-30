#include "drivetrain/drivetrain.h"

#include "tables/tables.h"
#include "constants/constants.h"

#include "drivetrain/logic.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "rev/CANSparkMax.h"

#include "RobotCompileModes.h" //set all robot modes here

#define DEG_TO_RAD(deg) ((deg / 180.0) * M_PI)

Tables tables;

//MAKE SURE TO REMOVE THIS AND REPLACE IT WITH HOW IT IS ACTUALY DECLARED FROM THE GYRO
double gyroFieldAngle = 0;

//constructor
Drivetrain::Drivetrain()
{
    tables.LogToNetworktable("Drivetrain initialised");
}

//set the heading of the robot as a whole, and set the individual wheels to the correct direction.
void Drivetrain::setHeadingRadians(double radians)
{
    Drivetrain::mRotationRadiansCentric = Drivetrain::fieldCentricToRobotAngle(radians, Drivetrain::mVelocityMeters, gyroFieldAngle);
    Drivetrain::mVelocityMetersCentric = Drivetrain::fieldCentricToRobotSpeed(Drivetrain::mVelocityMeters, radians, gyroFieldAngle);
    Drivetrain::mRotationRadians = radians;
}

void Drivetrain::setHeadingDegrees(double degrees)
{
    Drivetrain::setHeadingRadians(DEG_TO_RAD(degrees)); //converts degrees to radians
}

void Drivetrain::setVelocityMeters(double metersPerSecond)
{
    Drivetrain::mRotationRadiansCentric = Drivetrain::fieldCentricToRobotAngle(Drivetrain::mRotationRadians, metersPerSecond, gyroFieldAngle);
    Drivetrain::mVelocityMetersCentric = Drivetrain::fieldCentricToRobotSpeed(metersPerSecond, Drivetrain::mRotationRadians, gyroFieldAngle);
    Drivetrain::mVelocityMeters = metersPerSecond;
}

void Drivetrain::setVelocityFeet(double feetPerSecond)
{
    //feet to inches (12in in a foot), then to centimeters (2.52cm in an in), then to meters (100cm in one m) = 0.3024 ft in a meter
    double feetPerSecToMPS = feetPerSecond*0.3024;
    Drivetrain::setVelocityMeters(feetPerSecToMPS);
}

void Drivetrain::setSpinRadiansPerSecond(double radiansPerSecond){
    Drivetrain::mSpinRobotVelocity = radiansPerSecond;
}

void Drivetrain::setSpinDegreesPerSecond(double degreesPerSecond)
{
    Drivetrain::setSpinRadiansPerSecond(DEG_TO_RAD(degreesPerSecond)); //converts degrees to radians
}

void Drivetrain::setWheelMotorSpeeds(double *speeds){

}
