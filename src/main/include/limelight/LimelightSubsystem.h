#pragma once

#include <frc2/command/SubsystemBase.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include "networktables/NetworkTableInstance.h"

class LimelightSubsystem : public frc2::SubsystemBase
{
public:
    LimelightSubsystem();
    void changePipeToDrive();
    void changePipeToAim();
private:
    nt::NetworkTableEntry PipelineTable;
}