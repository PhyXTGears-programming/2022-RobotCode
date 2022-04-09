#pragma once

#include "intake/intake.h"
#include "shooter/shooter.h"
#include "drivetrain-swerve/SwerveDrive.h"

#include <frc2/command/SequentialCommandGroup.h>

class Auto {
    public:
        Auto(); // constructor

        static frc2::SequentialCommandGroup * MakeTwoCargoAutoNearWall (
            Intake * intake,
            Shooter * shooter,
            SwerveDrive * drive
        );

        static frc2::SequentialCommandGroup * MakeTwoCargoAuto (
            Intake * intake,
            Shooter * shooter,
            SwerveDrive * drive
        );

    private:
        //generally logic things that are not useful to have visable to the rest of the world
};
