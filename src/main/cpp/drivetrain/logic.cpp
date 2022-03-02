#include "drivetrain/logic.h"
#include "drivetrain/drivetrain.h"
#include "constants/constants.h"

#include <math.h>

double Drivetrain::fieldCentricToRobotAngle(double angle, double speed, double centerOfFieldAngle)
{
    return (-speed * sin(centerOfFieldAngle)) + (angle * cos(centerOfFieldAngle));
}

double Drivetrain::fieldCentricToRobotSpeed(double speed, double angle, double centerOfFieldAngle)
{
    return (speed * cos(centerOfFieldAngle)) + (angle * sin(centerOfFieldAngle));
}

std::vector<double> Drivetrain::getWheelSpeeds(double speed, double angle, double clockwiseSpin, double centerFieldAngle)
{
    double Y_Vector = (sin(angle)*speed);
    double X_Vector = (cos(angle)*speed);
    //not sure what the next four variables are exactly for, but they should be for calculating speed and angles
    double A = X_Vector - (clockwiseSpin * (constants::kWheelBase / constants::kDiameter));
    double B = X_Vector + (clockwiseSpin * (constants::kWheelBase / constants::kDiameter));
    double C = Y_Vector - (clockwiseSpin * (constants::kTrackWidth / constants::kDiameter));
    double D = Y_Vector + (clockwiseSpin * (constants::kTrackWidth / constants::kDiameter));

    double WheelSpeed1 = sqrt(pow(B, 2) + pow(C, 2));
    double WheelSpeed2 = sqrt(pow(B, 2) + pow(D, 2));
    double WheelSpeed3 = sqrt(pow(A, 2) + pow(D, 2));
    double WheelSpeed4 = sqrt(pow(A, 2) + pow(C, 2));

    double max = WheelSpeed1;

    if (WheelSpeed2 > max)
    {
        max = WheelSpeed2;
    }

    if (WheelSpeed3 > max)
    {
        max = WheelSpeed3;
    }

    if (WheelSpeed4 > max)
    {
        max = WheelSpeed4;
    }

    if (max > 1)
    {
        WheelSpeed1 /= max;
        WheelSpeed2 /= max;
        WheelSpeed3 /= max;
        WheelSpeed4 /= max;
    }
    
    std::vector<double> wheelSpeeds {WheelSpeed1, WheelSpeed2, WheelSpeed3, WheelSpeed4};

    return wheelSpeeds;
}

std::vector<double> Drivetrain::getWheelDirection(double speed, double angle, double clockwiseSpin, double centerFieldAngle)
{
    double Y_Vector = (sin(angle) * speed);
    double X_Vector = (cos(angle) * speed);
    //not sure what the next four variables are exactly for, but they should be for calculating speed and angles
    double A = X_Vector - (clockwiseSpin * (constants::kWheelBase / constants::kDiameter));
    double B = X_Vector + (clockwiseSpin * (constants::kWheelBase / constants::kDiameter));
    double C = Y_Vector - (clockwiseSpin * (constants::kTrackWidth / constants::kDiameter));
    double D = Y_Vector + (clockwiseSpin * (constants::kTrackWidth / constants::kDiameter));


    double WheelAngle1 = atan2(B, C);
    double WheelAngle2 = atan2(B, D);
    double WheelAngle3 = atan2(A, D);
    double WheelAngle4 = atan2(A, C);

    std::vector<double> wheelSpeeds {WheelAngle1, WheelAngle2, WheelAngle3, WheelAngle4};

    return wheelSpeeds;
}