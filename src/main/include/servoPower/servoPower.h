#pragma once

#include <frc2/command/SubsystemBase.h>
#include <constants/interfaces.h>
#include <frc/Relay.h>

class ServoPower : public frc2::SubsystemBase {
    public:
        void toggleServoPower(bool isOn);

    private:
        frc::Relay mServoPowerRelay {interfaces::kServoPowerRelay};
};