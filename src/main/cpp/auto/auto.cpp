#include "auto/auto.h"
#include "PID.h"

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
    const double kAcceptableError = 0.05;
    PID * turnPid = new PID(0.1, 0.0, 0.0, 0.1, 0.001);

    return new frc2::SequentialCommandGroup {
        ExtendIntakeCommand {intake},

        frc2::ParallelRaceGroup {
            RunIntakeCommand {intake},
            frc2::FunctionalCommand {
                [](){},
                [=](){
                    drive->setMotion(0, 0.2, 0.0);
                },
                [=](bool _interrupted){
                    drive->setMotion(0, 0, 0);
                },
                [](){ return false; },
                { drive }
            }.WithTimeout(1.35_s),
        },

        frc2::FunctionalCommand {
            [=](){
                turnPid->setTarget(drive->getHeading() - (160.0 * M_PI / 180.0));
            },
            [=](){
                drive->setMotion(0, 0, -turnPid->calculate(drive->getHeading()));
            },
            [=](bool _interrupted){
                drive->setMotion(0, 0, 0); //STOP
            },
            [=]() {
                return std::abs(turnPid->getError()) < kAcceptableError;

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
