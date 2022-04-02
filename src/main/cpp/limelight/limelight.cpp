#include "limelight/limelight.h"

limelight::limelight()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("limelight");
    mXAngle = table->GetEntry("tx");
    mYAngle = table->GetEntry("ty");
    mTargetFound = table->GetEntry("tv");
    oldestIndex = 0;
}

void limelight::Periodic()
{
    pullValuesFromNT();
}

void limelight::pullValuesFromNT()
{
    if (mTargetFound.GetBoolean(false))
    {
        angleArray[oldestIndex] = mXAngle.GetDouble(0);
        widthArray[oldestIndex] = mTargetWidth.GetDouble(0);
        oldestIndex = (oldestIndex + 1) % NUMBERS_COLLECTED_COUNT;
        numCollected += 1;
        if (numCollected >= NUMBERS_COLLECTED_COUNT)
        {
            currentAverage = getAverageValue();
        }
    }
}

double limelight::getAngle()
{
    return currentAverage;
}

double getAverageValue()
{
    double* originalAngleArray = angleArray;
    double* originalWidthArray = widthArray;

    int numberInNewArray = 0;
    // the array that will be averaged
    double newArray[NUMBERS_FINAL_AVERAGE_COUNT];
    double newArrayAngles[NUMBERS_FINAL_AVERAGE_COUNT];

    // initialized the values to 0
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