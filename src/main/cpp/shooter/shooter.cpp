#include "shooter/shooter.h"

void Shooter::runShooter (double speed) {
    mShooterMotor.Set(speed);
}

void Shooter::stopShooter () {
    runShooter(0.0);
}