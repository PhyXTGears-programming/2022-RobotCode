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

void limelight::pullValuesFromNT()
{
      if (mTargetFound.GetBoolean(false))
      {
            angleArray[oldestIndex] = mXAngle.GetDouble(0);
            widthArray[oldestIndex] = mTargetWidth.GetDouble(0);
            oldestIndex = (oldestIndex + 1) % NUMBERS_COLLECTED_COUNT;
            numCollected += 1;
            if(numCollected >= NUMBERS_COLLECTED_COUNT) {
                  currentAverage = getAverageValue();
            }
      }
}

double limelight::getAngle()
{
      return currentAverage;
}

double limelight::getAverageValue(){
      double *originalAngleArray = angleArray;
      double *originalWidthArray = widthArray;

      int numberInNerArray = 0;
      //the array that will be averaged
      double newArray[NUMBERS_FINAL_AVERAGE_COUNT];

      //initialized the values to 0
      double maxValues[NUMBERS_REJECTED_COUNT];

      //so we can keep track of where we need to start on the array for pushing off numbers
      //(and so we dont accidentally put 0s on the new array that did not come from the limelight)
      int numMaxValues = 0;

      //iterate through the array and push the numbers that fall off of the end of the podeum tot he array
      for(int i=0; i<=NUMBERS_COLLECTED_COUNT; i++){
            for(int h=0; i<=numMaxValues;h++){
                  if(originalWidthArray[i] >= maxValues[h]){
                        if(numMaxValues = NUMBERS_REJECTED_COUNT){
                              newArray[numberInNerArray] = originalWidthArray[i];
                              numberInNerArray += 1;
                        }
                        for(int j=0;j<(numMaxValues-h);j++){
                              maxValues[numMaxValues-j] = maxValues[numMaxValues-(j+1)];
                        }
                        maxValues[h] = originalWidthArray[i];
                        break;
                  }
            }
      }
}