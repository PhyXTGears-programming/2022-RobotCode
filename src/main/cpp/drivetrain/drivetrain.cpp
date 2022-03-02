#include "drivetrain/drivetrain.h"

#include "tables/tables.h"
#include "constants/constants.h"

#include "drivetrain/logic.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "rev/CANSparkMax.h"
#include "rev/REVLibError.h"

#include "RobotCompileModes.h" //set all robot modes here

#include <iostream>

#ifdef ROBOTCMH_PID_TUNING_MODE
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

//initialises networktable entries
nt::NetworkTableEntry PEntry;
nt::NetworkTableEntry IEntry;
nt::NetworkTableEntry DEntry;
nt::NetworkTableEntry FFEntry;
nt::NetworkTableEntry IZoneEntry;
nt::NetworkTableEntry PIDspeed;
nt::NetworkTableEntry PIDposition;
#endif

#define DEG_TO_RAD(deg) ((deg / 180.0) * M_PI)

Tables tables;

//MAKE SURE TO REMOVE THIS AND REPLACE IT WITH HOW IT IS ACTUALY DECLARED FROM THE GYRO
double gyroFieldAngle = 0;

//constructor
Drivetrain::Drivetrain()
{
    tables.LogToNetworktable("Drivetrain initialised");

#ifdef ROBOTCMH_PID_TUNING_MODE
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("datatable");

    PEntry = table->GetEntry("PTune");
    IEntry = table->GetEntry("ITune");
    DEntry = table->GetEntry("DTune");
    FFEntry = table->GetEntry("FFTune");
    IZoneEntry = table->GetEntry("IZoneTune");
    //will have to figure out the math to get that to be a speed of the wheel when I have more information
    //
    currentController.GetEncoder().SetVelocityConversionFactor(42);
    //we start with a 5.33:1 gear ratio from the swerve module, and
    //we have a 12:1 gear ratio for the output shaft on the motor
    // = 2688 encoder tick per revolution
    // currentController.GetEncoder().SetPositionConversionFactor(2688);
#endif

    //initial drive motor PID values
    Drivetrain::setPidValues(mPID_DriveMotor1, config.PID.Drive.motor1.k_P, config.PID.Drive.motor1.k_I, config.PID.Drive.motor1.k_D, config.PID.Drive.motor1.k_FF, config.PID.k_minOutput, config.PID.k_maxOutput, "DriveMotor1", config.PID.Drive.motor1.k_IZone);
    Drivetrain::setPidValues(mPID_DriveMotor2, config.PID.Drive.motor2.k_P, config.PID.Drive.motor2.k_I, config.PID.Drive.motor2.k_D, config.PID.Drive.motor2.k_FF, config.PID.k_minOutput, config.PID.k_maxOutput, "DriveMotor2", config.PID.Drive.motor2.k_IZone);
    Drivetrain::setPidValues(mPID_DriveMotor3, config.PID.Drive.motor3.k_P, config.PID.Drive.motor3.k_I, config.PID.Drive.motor3.k_D, config.PID.Drive.motor3.k_FF, config.PID.k_minOutput, config.PID.k_maxOutput, "DriveMotor3", config.PID.Drive.motor3.k_IZone);
    Drivetrain::setPidValues(mPID_DriveMotor4, config.PID.Drive.motor4.k_P, config.PID.Drive.motor4.k_I, config.PID.Drive.motor4.k_D, config.PID.Drive.motor4.k_FF, config.PID.k_minOutput, config.PID.k_maxOutput, "DriveMotor4", config.PID.Drive.motor4.k_IZone);
    //initial steering PID values
    Drivetrain::setPidValues(mPID_SteerMotor1, config.PID.Steer.motor1.k_P, config.PID.Steer.motor1.k_I, config.PID.Steer.motor1.k_D, config.PID.Steer.motor1.k_FF, config.PID.k_minOutput, config.PID.k_maxOutput, "SteerMotor1", config.PID.Steer.motor1.k_IZone);
    Drivetrain::setPidValues(mPID_SteerMotor2, config.PID.Steer.motor2.k_P, config.PID.Steer.motor2.k_I, config.PID.Steer.motor2.k_D, config.PID.Steer.motor2.k_FF, config.PID.k_minOutput, config.PID.k_maxOutput, "SteerMotor2", config.PID.Steer.motor2.k_IZone);
    Drivetrain::setPidValues(mPID_SteerMotor3, config.PID.Steer.motor3.k_P, config.PID.Steer.motor3.k_I, config.PID.Steer.motor3.k_D, config.PID.Steer.motor3.k_FF, config.PID.k_minOutput, config.PID.k_maxOutput, "SteerMotor3", config.PID.Steer.motor3.k_IZone);
    Drivetrain::setPidValues(mPID_SteerMotor4, config.PID.Steer.motor4.k_P, config.PID.Steer.motor4.k_I, config.PID.Steer.motor4.k_D, config.PID.Steer.motor4.k_FF, config.PID.k_minOutput, config.PID.k_maxOutput, "SteerMotor4", config.PID.Steer.motor4.k_IZone);
}

void Drivetrain::setPidValues(rev::SparkMaxPIDController PIDController, double k_P,
                              double k_I, double k_D, double k_FF,
                              double k_minValue, double k_maxValue, std::string motorReference, double k_IZone /*= 0*/)
{
    if (PIDController.SetP(k_P) != rev::REVLibError::kOk)
    {
        std::cout << "ERROR, P NOT SET SUCCESSFULLY ON MOTOR: " << motorReference << std::endl;
        PIDController.SetP(k_P);
    }
    if (PIDController.SetI(k_I) != rev::REVLibError::kOk)
    {
        std::cout << "ERROR, I NOT SET SUCCESSFULLY ON MOTOR: " << motorReference << std::endl;
        PIDController.SetI(k_I);
    }
    if (PIDController.SetD(k_D) != rev::REVLibError::kOk)
    {
        std::cout << "ERROR, D NOT SET SUCCESSFULLY ON MOTOR: " << motorReference << std::endl;
        PIDController.SetD(k_D);
    }
    if (PIDController.SetFF(k_FF) != rev::REVLibError::kOk)
    {
        std::cout << "ERROR, Feed Foreward NOT SET SUCCESSFULLY ON MOTOR: " << motorReference << std::endl;
        PIDController.SetFF(k_FF);
    }
    if (PIDController.SetOutputRange(k_minValue, k_maxValue) != rev::REVLibError::kOk)
    {
        std::cout << "ERROR, Output Range NOT SET SUCCESSFULLY ON MOTOR: " << motorReference << std::endl;
        PIDController.SetOutputRange(k_minValue, k_maxValue);
    }
    if (PIDController.SetIZone(k_IZone) != rev::REVLibError::kOk)
    {
        std::cout << "ERROR, I Zone NOT SET SUCCESSFULLY ON MOTOR: " << motorReference << std::endl;
        PIDController.SetIZone(k_IZone);
    }
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
    double feetPerSecToMPS = feetPerSecond * 0.3024;
    Drivetrain::setVelocityMeters(feetPerSecToMPS);
}

void Drivetrain::setSpinRadiansPerSecond(double radiansPerSecond)
{
    Drivetrain::mSpinRobotVelocity = radiansPerSecond;
}

void Drivetrain::setSpinDegreesPerSecond(double degreesPerSecond)
{
    Drivetrain::setSpinRadiansPerSecond(DEG_TO_RAD(degreesPerSecond)); //converts degrees to radians
}

void Drivetrain::setWheelMotorSpeeds(double *speeds)
{
    // m_driveMotor1
}

#ifdef ROBOTCMH_PID_TUNING_MODE
//grabs the values and sets them, while also setting speed of the motors to tables
void Drivetrain::tunePIDNetworktables()
{
    double k_P = PEntry.GetDouble(0.0);
    double k_I = IEntry.GetDouble(0.0);
    double k_D = DEntry.GetDouble(0.0);
    double k_FF = FFEntry.GetDouble(0.0);
    double k_IZone = IZoneEntry.GetDouble(0.0);

    Drivetrain::setPidValues(CurrentPIDController, k_P, k_I, k_D, k_FF, -1.0, 1.0, k_IZone);

    double speed = currentController.GetEncoder().GetVelocity();
    PIDspeed.SetDouble(speed);

    // double position = currentController.GetEncoder().GetPosition();
    // PIDposition.SetDouble(position);
}
#endif