#include "commands/drivetrain/DriveTeleopCommand.h"

#include "drivetrain/drivetrain.h"
#include <functional>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#include "constants/constants.h"

#define JOYSTICK_DEADZONE 0.3
#define MAKE_VALUE_FULL_RANGE(deadzonedInput) (1 / (1 - JOYSTICK_DEADZONE) * (deadzonedInput - std::copysign(JOYSTICK_DEADZONE, deadzonedInput)))
#define DEADZONE(input) ((fabs(input) < JOYSTICK_DEADZONE) ? 0.0 : MAKE_VALUE_FULL_RANGE(input))

#define HYPOTENUSE_PYTHAGORUS(x, y) (sqrt(((x) * (x)) + ((y) * (y))))

#define BOUNDS 1.0f

#define LEFT_X 0
#define LEFT_Y 1
#define RIGHT_X 2
#define RIGHT_Y 3

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
    grabJoystickValues();

    double m_LeftX = -DEADZONE(mJoystickAxis[LEFT_X]);
    double m_LeftY = -DEADZONE(mJoystickAxis[LEFT_Y]);
    double m_RightX = -DEADZONE(mJoystickAxis[RIGHT_X]);
    // double m_RightY = DEADZONE(mJoystickAxis[3]);
    angle = DriveTeleopCommand::theeta(m_LeftX, m_LeftY);
    radius = DriveTeleopCommand::cartToPolar(m_LeftX, m_LeftY);

    if ((m_LeftX == 0) && (m_LeftY == 0)) {
        radius = 0;
    } else {
        mDrivetrain->setHeadingRadians(angle);
        mDrivetrain->setVelocityMeters(radius);
        //mDrivetrain->setSpinDegreesPerSecond(m_RightX * 45);
        mDrivetrain->setWheels();
    }
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
    mJoystickAxis[LEFT_X] = mJoystick->GetLeftX();
    mJoystickAxis[LEFT_Y] = mJoystick->GetLeftY();
    mJoystickAxis[RIGHT_X] = mJoystick->GetRightX();
    mJoystickAxis[RIGHT_Y] = mJoystick->GetRightY();
}

// Take x and y components and compute angle in radians.
double DriveTeleopCommand::theeta(double x, double y)
{
    return atan2(y, x);
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