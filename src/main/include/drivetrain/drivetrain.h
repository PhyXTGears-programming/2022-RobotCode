#pragma once

#include "frc2/command/SubsystemBase.h"

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
    double *getWheelSpeeds(double speed, double angle, double clockwiseSpin, double centerFieldAngle);
    //gets the angle of the drive modules in radians (from -pi to +pi)
    double *getWheelDirection(double speed, double angle, double clockwiseSpin, double centerFieldAngle);
    //controll the motor controller speeds
    void setWheelMotorSpeeds(double *speeds);
};