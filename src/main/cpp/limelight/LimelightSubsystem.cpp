#include "limelight/LimelightSubsystem.h"

LimelightSubsystem::LimelightSubsystem(){
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("limelight");
    PipelineTable = table->GetEntry("pipeline");
}

void LimelightSubsystem::changePipeToAim(){
    PipelineTable->putNumber(0);
}

void LimelightSubsystem::changePipeToDrive(){
    PipelineTable->putNumber(1);
}