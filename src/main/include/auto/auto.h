#pragma once

#include <frc2/command/SequentialCommandGroup.h>

class Auto {
    public:
        Auto(); // constructor

        frc2::SequentialCommandGroup * Auto::MakeTwoCargoAuto (
            Intake * intake,
            Shooter * shooter,
            SwerveDrive * drive
        );

    private:
        //generally logic things that are not useful to have visable to the rest of the world
};
