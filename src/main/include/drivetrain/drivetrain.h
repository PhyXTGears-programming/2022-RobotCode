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
         * @brief used to turn the swerve modules to have a heading centric to the field in radians (put in terms of pi)
         * 
         * @param radians the radians of the wheel in respect to the center of the field
         */
    void setHeadingRadians(float radians);

    /**
         * @brief used to turn the swerve modules to have a heading centric to the field in degrees
         * 
         * @param degrees the radians of the wheel in respect to the center of the field
         */
    void setHeadingDegrees(double degrees);

    /**
         * @brief used to turn the swerve modules to have a heading centric to the field in degrees
         * 
         * @param degrees the radians of the wheel in respect to the center of the field
         */
    void setHeadingDegrees(float degrees);

    /**
         * @brief Set the Velocity of drive in meters per second (linear speed)
         * 
         * @param MetersPerSecond linear speed of motor
         */
    void setVelocityMeters(double MetersPerSecond);

    /**
         * @brief Set the Velocity of drive in meters per second (linear speed)
         * 
         * @param MetersPerSecond linear speed of motor
         */
    void setVelocityMeters(float MetersPerSecond);

    /**
         * @brief Set the Velocity of drive in meters per second (linear speed)
         * 
         * @param FeetPerSecond linear speed of motor
         */
    void setVelocityFeet(double FeetPerSecond);

    /**
         * @brief Set the Velocity of drive in meters per second (linear speed)
         * 
         * @param FeetPerSecond linear speed of motor
         */
    void setVelocityFeet(float FeetPerSecond);

    /**
         * @brief set the spinning speed of the robot in radians per second
         * 
         * @param radiansPerSecond
         */
    void setSpinRadiansPerSecond(double radiansPerSecond);

    /**
         * @brief set the spinning speed of the robot in radians per second
         * 
         * @param radiansPerSecond
         */
    void setSpinRadiansPerSecond(float radiansPerSecond);

    /**
         * @brief set the spinning speed of the robot in degrees per second
         * 
         * @param degreesPerSecond
         */
    void setSpinDegreesPerSecond(double degreesPerSecond);

    /**
         * @brief set the spinning speed of the robot in degrees per second
         * 
         * @param degreesPerSecond
         */
    void setSpinDegreesPerSecond(float degreesPerSecond);

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
    double rotationRadians;
    double velocityMeters;
};