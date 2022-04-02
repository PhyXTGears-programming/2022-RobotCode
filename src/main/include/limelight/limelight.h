#pragma once
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include "networktables/NetworkTableInstance.h"

//the number of numbers that  you want to collect from the limelight before it starts removing old numbers
#define NUMBERS_COLLECTED_COUNT 10
//the number of numbers you want to reject (currently the biggest width boxes are rejected only) before taking the average
#define NUMBERS_REJECTED_COUNT 3
#define NUMBERS_FINAL_AVERAGE_COUNT (NUMBERS_COLLECTED_COUNT - NUMBERS_REJECTED_COUNT)

class limelight {
public:
    limelight();
    double getAngle();
    bool targetFound();
    double PIDCalculate();
    void Periodic();
private:
    /**
     * @brief puls values from the network tables and sets the value to 0 on width if the target is not found, otherwise it will get the angle on the X-Axis.
     */
    void pullValuesFromNT();

    /**
     * @return the avarage value of the array discarding the elements with the widest target widths (done on a copy of the array)
     */
    double getAverageValue();

    /**
     * @brief calculates the largest value of the array because there is not one in the c++ library that returns the index
     * 
     * @return the index in the array of the max integer value
     */
    int maxArrayValueIndex(int array[NUMBERS_COLLECTED_COUNT]);

    nt::NetworkTableEntry mXAngle;
    nt::NetworkTableEntry mYAngle;
    nt::NetworkTableEntry mTargetFound;
    nt::NetworkTableEntry mTargetWidth;
    double angleArray[NUMBERS_COLLECTED_COUNT];
    double widthArray[NUMBERS_COLLECTED_COUNT];
    int oldestIndex = 0;
    int numCollected = 0;
    double currentAverage;
};