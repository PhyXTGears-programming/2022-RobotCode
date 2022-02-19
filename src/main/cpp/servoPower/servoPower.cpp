#include "servoPower/servoPower.h"

void ServoPower::toggleServoPower(bool isOn) {
    if (isOn) {
        mServoPowerRelay.Set(frc::Relay::Value::kOn);
    } else {
        mServoPowerRelay.Set(frc::Relay::Value::kOff);
    }
}