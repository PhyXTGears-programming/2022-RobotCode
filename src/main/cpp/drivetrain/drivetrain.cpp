#include "drivetrain/drivetrain.h"

#include "tables/tables.h"
#include "constants/constants.h"

#include "drivetrain/logic.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "rev/CANSparkMax.h"
#include "rev/REVLibError.h"

#include "cpptoml.h"

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

#define ROBOT_ONLY

#define DEG_TO_RAD(deg) ((deg / 180.0) * M_PI)

#define RAD_TO_DEG(deg) ((deg / M_PI) / 180)

#define DEG_TO_ROT(deg) (deg/360)

#define RAD_TO_ROT(rad) ((rad / M_PI) / 2)

//feet to inches (1ft = 12in), then to centimeters (1in = 2.54cm), then to meters (1cm = 0.01m) = 0.3048ft = 1meter
#define FEET_TO_METERS(feet) (feet * 0.3048)

//the motor revolutions and the wheel revolutions for steering are not the same, so we have to multiply it by this number to get a single revolution
#define MOTOR_TURN_CONVERSION_FACTOR(rad) ((RAD_TO_ROT(rad)) * 55.75) //was in radians, now in rotations (-pi to pi)

//radius of the swerve modeule drive wheel
#define DRIVE_WHEEL_RADIUS 0.0381 //done in meters
#define GEARING_RATIO 5.25
//the conversion from the floor speed to the motor's speed (outputted in meters per second, goes to morot RPMs)
//meters per second to rotations per second (of the wheel, omega) is MPS/radius (speed/radius)
//wheel-rps to motor-rps is Wheel-RPS/gear-ratio (omega-wheel*gear-ratio)
//motor-rps to motor-rpm is Motor-rps/60 (omega-motor*seconds-per-minute)
#define ROBOT_SPEED_TO_MOTOR_SPEED(speed) (((speed/DRIVE_WHEEL_RADIUS)*GEARING_RATIO)*60)

Tables tables;

//MAKE SURE TO REMOVE THIS AND REPLACE IT WITH HOW IT IS ACTUALY DECLARED FROM THE GYRO
double gyroFieldAngle = 0;

//constructor
Drivetrain::Drivetrain(std::shared_ptr<cpptoml::table> toml)
{
    
    // Drivetrain::mDriveMotor1.RestoreFactoryDefaults();
    // Drivetrain::mDriveMotor2.RestoreFactoryDefaults();
    // Drivetrain::mDriveMotor3.RestoreFactoryDefaults();
    // Drivetrain::mDriveMotor4.RestoreFactoryDefaults();
    // Drivetrain::mSteerMotor1.RestoreFactoryDefaults();
    // Drivetrain::mSteerMotor2.RestoreFactoryDefaults();
    // Drivetrain::mSteerMotor3.RestoreFactoryDefaults();
    // Drivetrain::mSteerMotor4.RestoreFactoryDefaults();

    //grabbing values from config.toml start
    tables.LogToNetworktable("Drivetrain initialised");

    Drivetrain::config.PID.Drive.motor1.k_P = toml->get_qualified_as<double>("PID.Drive.Motor1.k_P").value_or(config.PID.Drive.motor1.k_P);
    Drivetrain::config.PID.Drive.motor1.k_I = toml->get_qualified_as<double>("PID.Drive.Motor1.k_I").value_or(config.PID.Drive.motor1.k_I);
    Drivetrain::config.PID.Drive.motor1.k_D = toml->get_qualified_as<double>("PID.Drive.Motor1.k_D").value_or(config.PID.Drive.motor1.k_D);
    Drivetrain::config.PID.Drive.motor1.k_FF = toml->get_qualified_as<double>("PID.Drive.Motor1.k_FF").value_or(config.PID.Drive.motor1.k_FF);
    Drivetrain::config.PID.Drive.motor1.k_IZone = toml->get_qualified_as<double>("PID.Drive.Motor1.k_IZone").value_or(config.PID.Drive.motor1.k_IZone);

    Drivetrain::config.PID.Drive.motor2.k_P = toml->get_qualified_as<double>("PID.Drive.Motor2.k_P").value_or(config.PID.Drive.motor2.k_P);
    Drivetrain::config.PID.Drive.motor2.k_I = toml->get_qualified_as<double>("PID.Drive.Motor2.k_I").value_or(config.PID.Drive.motor2.k_I);
    Drivetrain::config.PID.Drive.motor2.k_D = toml->get_qualified_as<double>("PID.Drive.Motor2.k_D").value_or(config.PID.Drive.motor2.k_D);
    Drivetrain::config.PID.Drive.motor2.k_FF = toml->get_qualified_as<double>("PID.Drive.Motor2.k_FF").value_or(config.PID.Drive.motor2.k_FF);
    Drivetrain::config.PID.Drive.motor2.k_IZone = toml->get_qualified_as<double>("PID.Drive.Motor2.k_IZone").value_or(config.PID.Drive.motor2.k_IZone);

    Drivetrain::config.PID.Drive.motor3.k_P = toml->get_qualified_as<double>("PID.Drive.Motor3.k_P").value_or(config.PID.Drive.motor3.k_P);
    Drivetrain::config.PID.Drive.motor3.k_I = toml->get_qualified_as<double>("PID.Drive.Motor3.k_I").value_or(config.PID.Drive.motor3.k_I);
    Drivetrain::config.PID.Drive.motor3.k_D = toml->get_qualified_as<double>("PID.Drive.Motor3.k_D").value_or(config.PID.Drive.motor3.k_D);
    Drivetrain::config.PID.Drive.motor3.k_FF = toml->get_qualified_as<double>("PID.Drive.Motor3.k_FF").value_or(config.PID.Drive.motor3.k_FF);
    Drivetrain::config.PID.Drive.motor3.k_IZone = toml->get_qualified_as<double>("PID.Drive.Motor3.k_IZone").value_or(config.PID.Drive.motor3.k_IZone);

    Drivetrain::config.PID.Drive.motor4.k_P = toml->get_qualified_as<double>("PID.Drive.Motor4.k_P").value_or(config.PID.Drive.motor4.k_P);
    Drivetrain::config.PID.Drive.motor4.k_I = toml->get_qualified_as<double>("PID.Drive.Motor4.k_I").value_or(config.PID.Drive.motor4.k_I);
    Drivetrain::config.PID.Drive.motor4.k_D = toml->get_qualified_as<double>("PID.Drive.Motor4.k_D").value_or(config.PID.Drive.motor4.k_D);
    Drivetrain::config.PID.Drive.motor4.k_FF = toml->get_qualified_as<double>("PID.Drive.Motor4.k_FF").value_or(config.PID.Drive.motor4.k_FF);
    Drivetrain::config.PID.Drive.motor4.k_IZone = toml->get_qualified_as<double>("PID.Drive.Motor4.k_IZone").value_or(config.PID.Drive.motor4.k_IZone);

    Drivetrain::config.PID.Steer.motor1.k_P = toml->get_qualified_as<double>("PID.Steer.Motor1.k_P").value_or(config.PID.Steer.motor1.k_P);
    Drivetrain::config.PID.Steer.motor1.k_I = toml->get_qualified_as<double>("PID.Steer.Motor1.k_I").value_or(config.PID.Steer.motor1.k_I);
    Drivetrain::config.PID.Steer.motor1.k_D = toml->get_qualified_as<double>("PID.Steer.Motor1.k_D").value_or(config.PID.Steer.motor1.k_D);
    Drivetrain::config.PID.Steer.motor1.k_FF = toml->get_qualified_as<double>("PID.Steer.Motor1.k_FF").value_or(config.PID.Steer.motor1.k_FF);
    Drivetrain::config.PID.Steer.motor1.k_IZone = toml->get_qualified_as<double>("PID.Steer.Motor1.k_IZone").value_or(config.PID.Steer.motor1.k_IZone);

    Drivetrain::config.PID.Steer.motor2.k_P = toml->get_qualified_as<double>("PID.Steer.Motor2.k_P").value_or(config.PID.Steer.motor2.k_P);
    Drivetrain::config.PID.Steer.motor2.k_I = toml->get_qualified_as<double>("PID.Steer.Motor2.k_I").value_or(config.PID.Steer.motor2.k_I);
    Drivetrain::config.PID.Steer.motor2.k_D = toml->get_qualified_as<double>("PID.Steer.Motor2.k_D").value_or(config.PID.Steer.motor2.k_D);
    Drivetrain::config.PID.Steer.motor2.k_FF = toml->get_qualified_as<double>("PID.Steer.Motor2.k_FF").value_or(config.PID.Steer.motor2.k_FF);
    Drivetrain::config.PID.Steer.motor2.k_IZone = toml->get_qualified_as<double>("PID.Steer.Motor2.k_IZone").value_or(config.PID.Steer.motor2.k_IZone);

    Drivetrain::config.PID.Steer.motor3.k_P = toml->get_qualified_as<double>("PID.Steer.Motor3.k_P").value_or(config.PID.Steer.motor3.k_P);
    Drivetrain::config.PID.Steer.motor3.k_I = toml->get_qualified_as<double>("PID.Steer.Motor3.k_I").value_or(config.PID.Steer.motor3.k_I);
    Drivetrain::config.PID.Steer.motor3.k_D = toml->get_qualified_as<double>("PID.Steer.Motor3.k_D").value_or(config.PID.Steer.motor3.k_D);
    Drivetrain::config.PID.Steer.motor3.k_FF = toml->get_qualified_as<double>("PID.Steer.Motor3.k_FF").value_or(config.PID.Steer.motor3.k_FF);
    Drivetrain::config.PID.Steer.motor3.k_IZone = toml->get_qualified_as<double>("PID.Steer.Motor3.k_IZone").value_or(config.PID.Steer.motor3.k_IZone);

    Drivetrain::config.PID.Steer.motor4.k_P = toml->get_qualified_as<double>("PID.Steer.Motor4.k_P").value_or(config.PID.Steer.motor4.k_P);
    Drivetrain::config.PID.Steer.motor4.k_I = toml->get_qualified_as<double>("PID.Steer.Motor4.k_I").value_or(config.PID.Steer.motor4.k_I);
    Drivetrain::config.PID.Steer.motor4.k_D = toml->get_qualified_as<double>("PID.Steer.Motor4.k_D").value_or(config.PID.Steer.motor4.k_D);
    Drivetrain::config.PID.Steer.motor4.k_FF = toml->get_qualified_as<double>("PID.Steer.Motor4.k_FF").value_or(config.PID.Steer.motor4.k_FF);
    Drivetrain::config.PID.Steer.motor4.k_IZone = toml->get_qualified_as<double>("PID.Steer.Motor4.k_IZone").value_or(config.PID.Steer.motor4.k_IZone);

    
    Drivetrain::config.PID.k_maxOutput = toml->get_qualified_as<double>("PID.k_maxOutput").value_or(config.PID.k_maxOutput);
    Drivetrain::config.PID.k_minOutput = toml->get_qualified_as<double>("PID.k_minOutput").value_or(config.PID.k_minOutput);

    Drivetrain::config.Encoders.Encoder1GlobalOffset = toml->get_qualified_as<double>("EncoderSetpoint.Steer1GlobalOffset").value_or(config.Encoders.Encoder1GlobalOffset);
    Drivetrain::config.Encoders.Encoder2GlobalOffset = toml->get_qualified_as<double>("EncoderSetpoint.Steer2GlobalOffset").value_or(config.Encoders.Encoder2GlobalOffset);
    Drivetrain::config.Encoders.Encoder3GlobalOffset = toml->get_qualified_as<double>("EncoderSetpoint.Steer3GlobalOffset").value_or(config.Encoders.Encoder3GlobalOffset);
    Drivetrain::config.Encoders.Encoder4GlobalOffset = toml->get_qualified_as<double>("EncoderSetpoint.Steer4GlobalOffset").value_or(config.Encoders.Encoder4GlobalOffset);
    //grabbing values from config.toml end

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

    //makes all encoders absolutely positioned
    mSteerEncoder1.SetPositionToAbsolute();
    mSteerEncoder2.SetPositionToAbsolute();
    mSteerEncoder3.SetPositionToAbsolute();
    mSteerEncoder4.SetPositionToAbsolute();

    //makes the range from 0 to 360
    Drivetrain::mSteerEncoder1.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    Drivetrain::mSteerEncoder2.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    Drivetrain::mSteerEncoder3.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    Drivetrain::mSteerEncoder4.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);

    //makes sure it does not lose 0 point on reboot
    Drivetrain::mSteerEncoder1.ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition);
    Drivetrain::mSteerEncoder2.ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition);
    Drivetrain::mSteerEncoder3.ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition);
    Drivetrain::mSteerEncoder4.ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition);

    //sets the global offsets tuned from the toml file from pheonix tuner
    Drivetrain::mSteerEncoder1.ConfigMagnetOffset((config.Encoders.Encoder1GlobalOffset));
    Drivetrain::mSteerEncoder2.ConfigMagnetOffset((config.Encoders.Encoder2GlobalOffset));
    Drivetrain::mSteerEncoder3.ConfigMagnetOffset((config.Encoders.Encoder3GlobalOffset));
    Drivetrain::mSteerEncoder4.ConfigMagnetOffset((config.Encoders.Encoder4GlobalOffset));

    Drivetrain::InitialOffsetAngle1 = DEG_TO_RAD(Drivetrain::mSteerEncoder1.GetPosition());
    Drivetrain::InitialOffsetAngle2 = DEG_TO_RAD(Drivetrain::mSteerEncoder2.GetPosition());
    Drivetrain::InitialOffsetAngle3 = DEG_TO_RAD(Drivetrain::mSteerEncoder3.GetPosition());
    Drivetrain::InitialOffsetAngle4 = DEG_TO_RAD(Drivetrain::mSteerEncoder4.GetPosition());

    std::cout << DEG_TO_RAD(Drivetrain::mSteerEncoder1.GetPosition()) << std::endl;
    std::cout << DEG_TO_RAD(Drivetrain::mSteerEncoder2.GetPosition()) << std::endl;
    std::cout << DEG_TO_RAD(Drivetrain::mSteerEncoder3.GetPosition()) << std::endl;
    std::cout << DEG_TO_RAD(Drivetrain::mSteerEncoder4.GetPosition()) << std::endl;
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
    double feetPerSecToMPS = FEET_TO_METERS(feetPerSecond);
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

void Drivetrain::setWheelMotorSpeeds(std::vector<double> speeds)
{
    std::cout << "Speed 1: " << speeds[0] << std::endl;
    std::cout << "Speed 2: " << speeds[1] << std::endl;
    std::cout << "Speed 3: " << speeds[2] << std::endl;
    std::cout << "Speed 4: " << speeds[3] << std::endl;

    mPID_DriveMotor1.SetReference(ROBOT_SPEED_TO_MOTOR_SPEED(speeds[0] * constants::kMaxWheelSpeed), rev::CANSparkMax::ControlType::kVelocity);
    mPID_DriveMotor2.SetReference(ROBOT_SPEED_TO_MOTOR_SPEED(speeds[1] * constants::kMaxWheelSpeed), rev::CANSparkMax::ControlType::kVelocity);
    mPID_DriveMotor3.SetReference(ROBOT_SPEED_TO_MOTOR_SPEED(speeds[2] * constants::kMaxWheelSpeed), rev::CANSparkMax::ControlType::kVelocity);
    mPID_DriveMotor4.SetReference(ROBOT_SPEED_TO_MOTOR_SPEED(speeds[3] * constants::kMaxWheelSpeed), rev::CANSparkMax::ControlType::kVelocity);
}

void Drivetrain::setWheelMotorAngles(std::vector<double> angles)
{
    std::cout << "Angle 1: " << MOTOR_TURN_CONVERSION_FACTOR(angles[0]) << std::endl;
    std::cout << "Angle 2: " << MOTOR_TURN_CONVERSION_FACTOR(angles[1]) << std::endl;
    std::cout << "Angle 3: " << MOTOR_TURN_CONVERSION_FACTOR(angles[2]) << std::endl;
    std::cout << "Angle 4: " << MOTOR_TURN_CONVERSION_FACTOR(angles[3]) << std::endl;

    mPID_SteerMotor1.SetReference(MOTOR_TURN_CONVERSION_FACTOR(angles[1]), rev::CANSparkMax::ControlType::kPosition);//front left
    mPID_SteerMotor2.SetReference(MOTOR_TURN_CONVERSION_FACTOR(angles[0]), rev::CANSparkMax::ControlType::kPosition);//front right
    mPID_SteerMotor3.SetReference(MOTOR_TURN_CONVERSION_FACTOR(angles[3]), rev::CANSparkMax::ControlType::kPosition);//back right
    mPID_SteerMotor4.SetReference(MOTOR_TURN_CONVERSION_FACTOR(angles[2]), rev::CANSparkMax::ControlType::kPosition);//back left

    // std::cout << mSteerMotor1.GetEncoder().GetPosition();
    // std::cout << mSteerMotor2.GetEncoder().GetPosition();
    // std::cout << mSteerMotor3.GetEncoder().GetPosition();
    // std::cout << mSteerMotor4.GetEncoder().GetPosition();
}

void Drivetrain::turnOffMotors()
{
    mDriveMotor1.StopMotor();
    mDriveMotor2.StopMotor();
    mDriveMotor3.StopMotor();
    mDriveMotor4.StopMotor();
    mSteerMotor1.StopMotor();
    mSteerMotor2.StopMotor();
    mSteerMotor3.StopMotor();
    mSteerMotor4.StopMotor();
}

void Drivetrain::setWheels()
{
    Drivetrain::setWheelMotorAngles(Drivetrain::getWheelDirection(Drivetrain::mVelocityMetersCentric, Drivetrain::mRotationRadiansCentric, Drivetrain::mSpinRobotVelocity, 0));
    // Drivetrain::setWheelMotorAngles(std::vector<double>{0,0,0,0});
    Drivetrain::setWheelMotorSpeeds(Drivetrain::getWheelSpeeds(Drivetrain::mVelocityMetersCentric, Drivetrain::mRotationRadiansCentric, Drivetrain::mSpinRobotVelocity, 0));
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