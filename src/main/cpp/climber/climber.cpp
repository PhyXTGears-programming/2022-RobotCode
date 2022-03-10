#include "climber/climber.h"

#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber(std::shared_ptr<cpptoml::table> toml) {
    config.servo1.unlockPosition = toml->get_qualified_as<double>("servo1.unlockPosition").value_or(0.0);
    config.servo1.lockPosition = toml->get_qualified_as<double>("servo1.lockPosition").value_or(0.0);

    config.servo2.unlockPosition = toml->get_qualified_as<double>("servo2.unlockPosition").value_or(0.0);
    config.servo2.lockPosition = toml->get_qualified_as<double>("servo2.lockPosition").value_or(0.0);

    config.extendSpeed = toml->get_qualified_as<double>("extendSpeed").value_or(0.5);
    config.retractSpeed = toml->get_qualified_as<double>("retractSpeed").value_or(-0.5);
    config.inchesPerRevolution = toml->get_qualified_as<double>("inchesPerRevolution").value_or(0.128325);
    config.friction.innerStaticFriction = toml->get_qualified_as<double>("innerStaticFriction").value_or(0.0);
    config.friction.outerStaticFriction = toml->get_qualified_as<double>("outerStaticFriction").value_or(0.0);
    config.friction.innerStaticFrictionWithLoad = toml->get_qualified_as<double>("innerStaticFrictionWithLoad").value_or(0.0);
    config.friction.outerStaticFrictionWithLoad = toml->get_qualified_as<double>("outerStaticFrictionWithLoad").value_or(0.0);
}

// void Climber::Periodic() {
//     frc::SmartDashboard::PutNumber("Climb motor current", mInnerHookMotor1.GetOutputCurrent());

//     frc::SmartDashboard::PutNumber("Clmb In L Pos", getInner1Position());
//     frc::SmartDashboard::PutNumber("Clmb In R Pos", getInner2Position());

//     frc::SmartDashboard::PutNumber("Clmb Out L Pos", getOuter1Position());
//     frc::SmartDashboard::PutNumber("Clmb Out R Pos", getOuter2Position());

//     frc::SmartDashboard::PutNumber("Clmb In Angle", getInnerAngle());
//     frc::SmartDashboard::PutNumber("Clmb Out Angle", getOuterAngle());
// }

