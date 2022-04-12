#include "limelight/limelight.h"

limelight::limelight()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    mTable = inst.GetTable("limelight");
    oldestIndex = 0;
    pidController = new PID(PIDValues.P, PIDValues.I, PIDValues.D, PIDValues.FF, PIDValues.AcceptableError, PIDValues.Min, PIDValues.Max, PIDValues.IZone, PIDValues.timePeriod);
    pidController->setTarget(0.0);
}

void limelight::Periodic()
{
    pullValuesFromNT();
}

void limelight::pullValuesFromNT()
{
    if (mTable->GetNumber("tv", 0.0) == 1.0)
    {
        angleArray[oldestIndex] = mTable->GetNumber("tx", 0.0);
        widthArray[oldestIndex] = mTable->GetNumber("tw", 0.0);
        oldestIndex = (oldestIndex + 1) % NUMBERS_COLLECTED_COUNT;
        numCollected += 1;
        if (numCollected >= NUMBERS_COLLECTED_COUNT)
        {
            currentAverage = getAverageValue();
            numCollected -= 1;
        }
    }
}

double limelight::getAngle()
{
    return currentAverage;
}

double limelight::getAverageValue()
{
    double* originalAngleArray = angleArray;
    double* originalWidthArray = widthArray;

    int numberInNewArray = 0;
    // the array that will be averaged
    double newArray[NUMBERS_FINAL_AVERAGE_COUNT];
    double newArrayAngles[NUMBERS_FINAL_AVERAGE_COUNT];

    // temporary array for storing the largest values of the array
    double maxValuesWidths[NUMBERS_REJECTED_COUNT];
    double maxValuesAngles[NUMBERS_REJECTED_COUNT];

    // so we can keep track of where we need to start on the array for pushing off numbers
    //(and so we dont accidentally put 0s on the new array that did not come from the limelight)
    int numMaxValues = 0;


    bool passedMaxOverflow = false;

    // iterate through the array and push the numbers that fall off of the end of the podeum tot he array
    for (int i = 0; i < NUMBERS_COLLECTED_COUNT; i++)
    {
        for (int h = 0; h <= numMaxValues; h++)
        {
            if (originalWidthArray[i] >= maxValuesWidths[h])
            {
                if ((numMaxValues == (NUMBERS_REJECTED_COUNT-1)) && passedMaxOverflow)
                {
                    newArray[numberInNewArray] = maxValuesWidths[numMaxValues];
                    newArrayAngles[numberInNewArray] = maxValuesAngles[numMaxValues];
                    numberInNewArray += 1;
                }
                else if ((numMaxValues == (NUMBERS_REJECTED_COUNT-1)) && !passedMaxOverflow) {
                    passedMaxOverflow = true;
                }
                for (int j = 0; j < (numMaxValues - h); j++)
                {
                    maxValuesWidths[numMaxValues - j] = maxValuesWidths[numMaxValues - (j + 1)];
                    maxValuesAngles[numMaxValues - j] = maxValuesAngles[numMaxValues - (j + 1)];
                }
                maxValuesWidths[h] = originalWidthArray[i];
                maxValuesAngles[h] = originalAngleArray[i];
                numMaxValues += 1;
                if (numMaxValues >= NUMBERS_REJECTED_COUNT)
                {
                    numMaxValues -= 1;
                }
                break;
            }
            else if (h == (NUMBERS_REJECTED_COUNT - 1)) {
                newArray[numberInNewArray] = originalWidthArray[i];
                newArrayAngles[numberInNewArray] = originalAngleArray[i];
                numberInNewArray += 1;
            }
            else {
                continue;
            }
        }
    }

    double runTot = 0;
    for (int a = 0; a < NUMBERS_FINAL_AVERAGE_COUNT; a++) {
        runTot += newArrayAngles[a];
    }
    return runTot / NUMBERS_FINAL_AVERAGE_COUNT;

}

double limelight::PIDCalculate(){
    return pidController->calculate(currentAverage);
}

void limelight::finishAim(){
    pidController->reset();

    //reset values to 0 in the entire array when done aiming.
    //currently TBD on if we need to or not (such as if we are only collecting numbers while aiming)
    //std::fill(angleArray, angleArray + NUMBERS_COLLECTED_COUNT, 0);
    //std::fill(widthArray, widthArray + NUMBERS_COLLECTED_COUNT, 0);
}