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

    //initial drive motor PID values
    Drivetrain::setPidValues(m_PID_DriveMotor1, k_PID_DriveMotor1_P, k_PID_DriveMotor1_I, k_PID_DriveMotor1_D, k_PID_DriveMotor1_F, k_minOutput, k_MaxOutput);
    Drivetrain::setPidValues(m_PID_DriveMotor2, k_PID_DriveMotor2_P, k_PID_DriveMotor2_I, k_PID_DriveMotor2_D, k_PID_DriveMotor2_F, k_minOutput, k_MaxOutput);
    Drivetrain::setPidValues(m_PID_DriveMotor3, k_PID_DriveMotor3_P, k_PID_DriveMotor3_I, k_PID_DriveMotor3_D, k_PID_DriveMotor3_F, k_minOutput, k_MaxOutput);
    Drivetrain::setPidValues(m_PID_DriveMotor4, k_PID_DriveMotor4_P, k_PID_DriveMotor4_I, k_PID_DriveMotor4_D, k_PID_DriveMotor4_F, k_minOutput, k_MaxOutput);
    //initial steering PID values
    Drivetrain::setPidValues(m_PID_SteerMotor1, k_PID_SteerMotor1_P, k_PID_SteerMotor1_I, k_PID_SteerMotor1_D, k_PID_SteerMotor1_F, k_minOutput, k_MaxOutput);
    Drivetrain::setPidValues(m_PID_SteerMotor2, k_PID_SteerMotor2_P, k_PID_SteerMotor2_I, k_PID_SteerMotor2_D, k_PID_SteerMotor2_F, k_minOutput, k_MaxOutput);
    Drivetrain::setPidValues(m_PID_SteerMotor3, k_PID_SteerMotor3_P, k_PID_SteerMotor3_I, k_PID_SteerMotor3_D, k_PID_SteerMotor3_F, k_minOutput, k_MaxOutput);
    Drivetrain::setPidValues(m_PID_SteerMotor4, k_PID_SteerMotor4_P, k_PID_SteerMotor4_I, k_PID_SteerMotor4_D, k_PID_SteerMotor4_F, k_minOutput, k_MaxOutput);
}

void Drivetrain::setPidValues(rev::SparkMaxPIDController PIDController, double k_P,
                              double k_I, double k_D, double k_FF,
                              double k_minValue, double k_maxValue, double k_IZone /*= 0.0*/)
{
    PIDController.SetP(k_P);
    PIDController.SetI(k_I);
    PIDController.SetD(k_D);
    PIDController.SetFF(k_FF);
    PIDController.SetOutputRange(k_minValue, k_maxValue);
    PIDController.SetIZone(k_IZone);
}

//set the heading of the robot as a whole, and set the individual wheels to the correct direction.
void Drivetrain::setHeadingRadians(double radians)
{
    Drivetrain::m_RotationRadiansCentric = Drivetrain::fieldCentricToRobotAngle(radians, Drivetrain::m_VelocityMeters, gyroFieldAngle);
    Drivetrain::m_VelocityMetersCentric = Drivetrain::fieldCentricToRobotSpeed(Drivetrain::m_VelocityMeters, radians, gyroFieldAngle);
    Drivetrain::m_RotationRadians = radians;
}

void Drivetrain::setHeadingDegrees(double degrees)
{
    Drivetrain::setHeadingRadians(DEG_TO_RAD(degrees)); //converts degrees to radians
}

void Drivetrain::setVelocityMeters(double metersPerSecond)
{
    Drivetrain::m_RotationRadiansCentric = Drivetrain::fieldCentricToRobotAngle(Drivetrain::m_RotationRadians, metersPerSecond, gyroFieldAngle);
    Drivetrain::m_VelocityMetersCentric = Drivetrain::fieldCentricToRobotSpeed(metersPerSecond, Drivetrain::m_RotationRadians, gyroFieldAngle);
    Drivetrain::m_VelocityMeters = metersPerSecond;
}

void Drivetrain::setVelocityFeet(double feetPerSecond)
{
    //feet to inches (12in in a foot), then to centimeters (2.52cm in an in), then to meters (100cm in one m) = 0.3024 ft in a meter
    double feetPerSecToMPS = feetPerSecond * 0.3024;
    Drivetrain::setVelocityMeters(feetPerSecToMPS);
}

void Drivetrain::setSpinRadiansPerSecond(double radiansPerSecond)
{
    Drivetrain::m_SpinRobotVelocity = radiansPerSecond;
}

void Drivetrain::setSpinDegreesPerSecond(double degreesPerSecond)
{
    Drivetrain::setSpinRadiansPerSecond(DEG_TO_RAD(degreesPerSecond)); //converts degrees to radians
}

void Drivetrain::setWheelMotorSpeeds(double *speeds)
{
    // m_driveMotor1
}


}
