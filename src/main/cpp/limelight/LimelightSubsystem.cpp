#include "limelight/LimelightSubsystem.h"

LimelightSubsystem::LimelightSubsystem(){
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("limelight");
    PipelineTable = table->GetEntry("pipeline");
}

void LimelightSubsystem::changePipeToAim(){
    PipelineTable.SetDouble(0);
}

void LimelightSubsystem::changePipeToDrive(){
    PipelineTable.SetDouble(1);
}