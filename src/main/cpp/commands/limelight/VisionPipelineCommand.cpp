#include "commands/limelight/VisionPipelineCommand.h"

VisionPipelineCommand::VisionPipelineCommand(LimelightSubsystem * limelightSubsystem){
    AddRequirements(limelightSubsystem);
    mLimelightSubsystem = limelightSubsystem;
}

void VisionPipelineCommand::Initialize(){
    mLimelightSubsystem->changePipeToAim();
}

void VisionPipelineCommand::End(bool interrupted){
    mLimelightSubsystem->changePipeToDrive();
}

bool VisionPipelineCommand::IsFinished(){
    return false;
}