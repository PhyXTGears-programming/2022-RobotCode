#pragma once

#include <string>
#include "frc2/command/SubsystemBase.h"

class Tables : public frc2::SubsystemBase {
    public:
        Tables(); // constructor

        /**
         * @brief log the input to the networktables one line at a time
         * 
         * @param LoggedText string of text in which you wish to be logged to the networktables
         */
        void LogToNetworktable(std::string LoggedText);

        //other publically available functions

    private:
        //private functions
        //typically logic functions
};