#pragma once

#include "frc2/command/SubsystemBase.h"

#include "rev/CANSparkMax.h"
#include "constants/constants.h"
#include "constants/interfaces.h"

#include "RobotCompileModes.h"

class Drivetrain : public frc2::SubsystemBase
{
public:
    Drivetrain(); // constructor

    /**
         * @brief used to turn the swerve modules to have a heading centric to the field in radians (put in terms of pi)
         * 
         * @param radians the radians of the wheel in respect to the center of the field
         */
    void setHeadingRadians(double radians);

    /**
         * @brief used to turn the swerve modules to have a heading centric to the field in degrees
         * 
         * @param degrees the radians of the wheel in respect to the center of the field
         */
    void setHeadingDegrees(double degrees);

    /**
         * @brief Set the Velocity of drive in meters per second (linear speed)
         * 
         * @param MetersPerSecond linear speed of motor
         */
    void setVelocityMeters(double metersPerSecond);

    /**
         * @brief Set the Velocity of drive in meters per second (linear speed)
         * 
         * @param FeetPerSecond linear speed of motor
         */
    void setVelocityFeet(double feetPerSecond);

    /**
         * @brief set the spinning speed of the robot in radians per second
         * 
         * @param radiansPerSecond
         */
    void setSpinRadiansPerSecond(double radiansPerSecond);

    /**
         * @brief set the spinning speed of the robot in degrees per second
         * 
         * @param degreesPerSecond
         */
    void setSpinDegreesPerSecond(double degreesPerSecond);

    /**
         * @brief Get the current heading of the robot (in terms of pi radians)
         * 
         * @return the heading in radians of the robot
         */
    double getHeadingRadians();

    /**
         * @brief Get the current heading of the robot
         * 
         * @return the heading in degrees of the robot
         */
    double getHeadingDegrees();

    /**
         * @brief Get the current velocity of the robot
         * 
         * @return the velocity in m/s of the robot
         */
    double getVelocityMeters();

    /**
         * @brief Get the current velocity of the robot
         * 
         * @return the velocity in ft/s of the robot
         */
    double getVelocityfeet();

    /**
         * @brief Get the current Spin Degrees Per Second
         * 
         * @return the spinning velovity in degrees per second
         */
    double getSpinRadiansPerSecond();

    /**
         * @brief Get the current Spin Degrees Per Second
         * 
         * @return the spinning velovity in degrees per second
         */
    double getSpinDegreesPerSecond();

    /**
         * @brief get current PID values and post relevent data to the network tables
         * 
         */
    void tunePIDNetworktables();

private:
    //generally logic things that are not useful to have visable to the rest of the world

    //the current values of the robot before field centric calculations
    double mRotationRadians;
    double mVelocityMeters;
    double mSpinRobotVelocity;

    double mRotationRadiansCentric;
    double mVelocityMetersCentric;

    //takes the angle from the controller and makes it local to the robot front
    double fieldCentricToRobotAngle(double angle, double speed, double centerOfFieldAngle);
    //takes the speed from the controller and makes it local to the robot
    double fieldCentricToRobotSpeed(double speed, double angle, double centerOfFieldAngle);
    //gets the percent of the speed in which the robot should be going from max (they are normalised so that it will not go above 1)
    std::vector<double> getWheelSpeeds(double speed, double angle, double clockwiseSpin, double centerFieldAngle);
    //gets the angle of the drive modules in radians (from -pi to +pi)
    std::vector<double> getWheelDirection(double speed, double angle, double clockwiseSpin, double centerFieldAngle);
    //control the motor controller speeds
    void setWheelMotorSpeeds(double *speeds);

    /**
     * @brief Set the PID Values of the motor
     * 
     * @param PIDController the motor controller PID controller object
     * @param k_P the P value to set it to
     * @param k_I the I value to set it to
     * @param k_D the D value to set it to
     * @param k_FF the Feed foreward value to set it to
     * @param k_minValue the minimum output value of the PID loop
     * @param k_maxValue the miximum output value of the PID loop
     * @param k_IZone the I zone value to set it to (defaults to 0)
     */
    void setPidValues(rev::SparkMaxPIDController PIDController, double k_P, double k_I, double k_D, double k_FF, double k_minValue, double k_maxValue, double k_IZone = 0.0);

    //initialised the motors

    rev::CANSparkMax mDriveMotor1{interfaces::kDrive1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mDriveMotor2{interfaces::kDrive2, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mDriveMotor3{interfaces::kDrive3, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mDriveMotor4{interfaces::kDrive4, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mSteerMotor1{interfaces::kSteer1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mSteerMotor2{interfaces::kSteer2, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mSteerMotor3{interfaces::kSteer3, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mSteerMotor4{interfaces::kSteer4, rev::CANSparkMax::MotorType::kBrushless};

    //initialised PID controls

    rev::SparkMaxPIDController mPID_DriveMotor1 = Drivetrain::mDriveMotor1.GetPIDController();
    rev::SparkMaxPIDController mPID_DriveMotor2 = Drivetrain::mDriveMotor2.GetPIDController();
    rev::SparkMaxPIDController mPID_DriveMotor3 = Drivetrain::mDriveMotor3.GetPIDController();
    rev::SparkMaxPIDController mPID_DriveMotor4 = Drivetrain::mDriveMotor4.GetPIDController();
    rev::SparkMaxPIDController mPID_SteerMotor1 = Drivetrain::mSteerMotor1.GetPIDController();
    rev::SparkMaxPIDController mPID_SteerMotor2 = Drivetrain::mSteerMotor2.GetPIDController();
    rev::SparkMaxPIDController mPID_SteerMotor3 = Drivetrain::mSteerMotor3.GetPIDController();
    rev::SparkMaxPIDController mPID_SteerMotor4 = Drivetrain::mSteerMotor4.GetPIDController();

    //global constants for the motors
    const double k_MaxOutput = 1.0;
    const double k_minOutput = -1.0;

    //hardcoded PID values (temporary)
    //k_ for the member constants to differentiate betweek global constants of k
    const double k_PID_DriveMotor1_P = 0.0;
    const double k_PID_DriveMotor1_I = 0.0;
    const double k_PID_DriveMotor1_D = 0.0;
    const double k_PID_DriveMotor1_F = 0.0;

    const double k_PID_DriveMotor2_P = 0.0;
    const double k_PID_DriveMotor2_I = 0.0;
    const double k_PID_DriveMotor2_D = 0.0;
    const double k_PID_DriveMotor2_F = 0.0;

    const double k_PID_DriveMotor3_P = 0.0;
    const double k_PID_DriveMotor3_I = 0.0;
    const double k_PID_DriveMotor3_D = 0.0;
    const double k_PID_DriveMotor3_F = 0.0;

    const double k_PID_DriveMotor4_P = 0.0;
    const double k_PID_DriveMotor4_I = 0.0;
    const double k_PID_DriveMotor4_D = 0.0;
    const double k_PID_DriveMotor4_F = 0.0;

    const double k_PID_SteerMotor1_P = 0.0;
    const double k_PID_SteerMotor1_I = 0.0;
    const double k_PID_SteerMotor1_D = 0.0;
    const double k_PID_SteerMotor1_F = 0.0;

    const double k_PID_SteerMotor2_P = 0.0;
    const double k_PID_SteerMotor2_I = 0.0;
    const double k_PID_SteerMotor2_D = 0.0;
    const double k_PID_SteerMotor2_F = 0.0;

    const double k_PID_SteerMotor3_P = 0.0;
    const double k_PID_SteerMotor3_I = 0.0;
    const double k_PID_SteerMotor3_D = 0.0;
    const double k_PID_SteerMotor3_F = 0.0;

    const double k_PID_SteerMotor4_P = 0.0;
    const double k_PID_SteerMotor4_I = 0.0;
    const double k_PID_SteerMotor4_D = 0.0;
    const double k_PID_SteerMotor4_F = 0.0;

#ifdef ROBOTCMH_PID_TUNING_MODE
    rev::CANSparkMax &currentController = Drivetrain::mDriveMotor1;
    rev::SparkMaxPIDController CurrentPIDController = Drivetrain::currentController.GetPIDController();
#endif
};