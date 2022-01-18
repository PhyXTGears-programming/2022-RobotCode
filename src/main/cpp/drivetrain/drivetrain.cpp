#include "drivetrain/drivetrain.h"

#include "tables/tables.h"
#include "constants/constants.h"

Tables tables;

//constructor
Drivetrain::Drivetrain(){
    tables.LogToNetworktable("Drivetrain initialised");
}

//set the heading of the robot as a whole, and set the individual wheels to the correct direction.
void Drivetrain::setHeadingRadians(double radians){
}