#include "auto/auto.h"

#include "commands/intake/ExtendIntake.h"
#include "commands/intake/RetractIntake.h"
#include "commands/intake/RunIntake.h"

#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/FunctionalCommand.h>

#include <cmath>

frc2::SequentialCommandGroup * Auto::MakeTwoCargoAuto (
    Intake * intake,
    Shooter * shooter,
    SwerveDrive * drive
) {
    double * targetGyroPosition = new double(0.0);
    const double kAcceptableError = 0.05;

    return new frc2::SequentialCommandGroup {
        ExtendIntakeCommand {intake},

        frc2::ParallelRaceGroup {
            RunIntakeCommand {intake},
            frc2::FunctionalCommand {
                [](){},
                [=](){
                    drive->setMotion(0, 0.1, 0);
                },
                [=](bool _interrupted){
                    drive->setMotion(0, 0, 0);
                },
                [](){ return false; },
                { drive }
            }.WithTimeout(1.2_s),
        },

        frc2::FunctionalCommand {
            [=](){
                *targetGyroPosition = drive->getHeading() + M_PI;
            },
            [=](){
                drive->setMotion(0, 0, 0.25);
            },
            [=](bool _interrupted){
                drive->setMotion(0, 0, 0); //STOP
            },
            [=](){
                double currentPosition = drive->getHeading();
                return std::abs(*targetGyroPosition - currentPosition) < kAcceptableError;

            },
            { drive }
        },

        frc2::ParallelRaceGroup {
            RunIntakeCommand {intake},
            frc2::StartEndCommand {
                [&]() { shooter->shootFar(); },
                [&]() { shooter->stopShooter(); },
                { shooter }
            }.WithTimeout(8_s)
        },

        RetractIntakeCommand {intake}
    };
}
