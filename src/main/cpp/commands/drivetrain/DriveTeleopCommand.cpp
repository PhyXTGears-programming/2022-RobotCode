#include "commands/drivetrain/DriveTeleopCommand.h"

#include "drivetrain/drivetrain.h"
#include <functional>
#include <iostream>
#include <math.h>

#include "constants/constants.h"

#define JOYSTICK_DEADZONE 0.1
#define MAKE_VALUE_FULL_RANGE(deadzonedInput) (1 / (1 - JOYSTICK_DEADZONE) * (deadzonedInput - std::copysign(JOYSTICK_DEADZONE, deadzonedInput)))
#define DEADZONE(input) ((fabs(input) < JOYSTICK_DEADZONE) ? 0.0 : MAKE_VALUE_FULL_RANGE(input))

#define HYPOTENUSE_PYTHAGORUS(x, y) (sqrt((x * x) + (y * y)))

#define BOUNDS 1.0f

DriveTeleopCommand::DriveTeleopCommand(Drivetrain *drivetrain, frc::XboxController *driverController)
{
    AddRequirements(drivetrain);
    mDrivetrain = drivetrain;
    mJoystick = driverController;
}

void DriveTeleopCommand::Initialize()
{
    std::cout << "init" << std::endl;
    mDrivetrain->setVelocityMeters(0);
    mDrivetrain->setHeadingRadians(0);
    mDrivetrain->setSpinDegreesPerSecond(0);
    mDrivetrain->setWheels();
    //reset encoder position
    //initialise
}

void DriveTeleopCommand::Execute()
{
    std::cout << "execute" << std::endl;
    grabJoystickValues();

    double m_LeftX = DEADZONE(mJoystickAxis[0]);
    double m_LeftY = DEADZONE(mJoystickAxis[1]);
    double m_RightX = DEADZONE(mJoystickAxis[2]);
    // double m_RightY = DEADZONE(mJoystickAxis[3]);
    angle = DriveTeleopCommand::theeta(m_LeftX, m_LeftY);
    radius = DriveTeleopCommand::cartToPolar(m_LeftX, m_LeftY);
    if (isnan(angle))
    {
        angle = prevGoodAngle;
    }
    else
    {
        prevGoodAngle = angle;
    }
    if ((m_LeftX == 0) && (m_LeftY == 0))
    {
        radius = 0;
    }
    mDrivetrain->setHeadingDegrees(angle);
    mDrivetrain->setVelocityMeters(radius);
    mDrivetrain->setSpinDegreesPerSecond(m_RightX * 45);
    mDrivetrain->setWheels();
}

void DriveTeleopCommand::End(bool interrupted)
{

    std::cout << "interupted" << std::endl;
    //stopping code
}

bool DriveTeleopCommand::IsFinished()
{
    //finished code
    //never say it is finished because the drivetrain will never need to stop moving in teleop
    return false;
}

void DriveTeleopCommand::grabJoystickValues()
{
    mJoystickAxis[0] = mJoystick->GetLeftX();
    mJoystickAxis[1] = mJoystick->GetLeftY();
    mJoystickAxis[2] = mJoystick->GetRightX();
    mJoystickAxis[3] = mJoystick->GetRightY();
}

double DriveTeleopCommand::theeta(double x, double y)
{
    double theta = atan(y / x);
    double angle = theta * (180.0 / M_PI);

    if ((x < 0) && (y > 0))
    {
        angle += 180;
    }
    else if ((x < 0) && (y < 0))
    {
        angle -= 180;
    }
    return angle;
}

float DriveTeleopCommand::cartToPolar(float inputX, float inputY)
{
    //Convert x-y cartesian coordinate to a polar distance
    //from center.

    float edgePointX, edgePointY;

    //First calculate the slope
    //if one of the values is 0, respond appropriately so that
    //we don't have divide by 0 errors and such
    if (inputY == 0)
    {
        //If there is no Y factor
        if (inputX > 0)
            edgePointX = BOUNDS;
        else if (inputX < 0)
            edgePointX = -BOUNDS;
        else
            //Return 0 if both x and y inputs are 0
            return 0;

        edgePointY = 0;
    }
    else if (inputX == 0)
    {
        //If there is no X factor
        if (inputY > 0)
            edgePointY = BOUNDS;
        else if (inputY < 0)
            edgePointY = -BOUNDS;
        else
            //Return 0 if both x and y inputs are 0
            return 0;

        edgePointX = 0;
    }
    else
    {
        //If we have no 0 in x or y input, calculate slope
        //normally
        float slope = inputY / inputX;

        //Project a point along the line of our actual point
        //using the slope

        if (slope >= 1.0f || slope <= -1.0f)
        {
            //If the point is on the top or bottom bounds
            edgePointX = BOUNDS / slope;
            edgePointY = BOUNDS;
        }
        else
        {
            //If the point is on the side bounds
            edgePointX = BOUNDS;
            edgePointY = BOUNDS * slope;
        }
    }

    //Calculate the length of the line from the center
    //to our edge point
    float edgeDistance = sqrt(pow(fabs(edgePointX), 2) + pow(fabs(edgePointY), 2));

    //Calculate length of line from the center to our actual point
    float inputDistance = sqrt(pow(((float)fabs(inputX)), 2) + pow(((float)fabs(inputY)), 2));

    //Return the "polar distance" by finding the ratio of these two lines
    //and multiplying by the bound
    return BOUNDS * (inputDistance / edgeDistance);
}