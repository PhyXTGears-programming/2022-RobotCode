#include "commands/limelight/AimingVisionCommand.h"

AimingVisionCommand::AimingVisionCommand(LimelightSubsystem * limelightSubsystem){
    AddRequirements(limelightSubsystem);
    mLimelightSubsystem = limelightSubsystem;
}

void AimingVisionCommand::Initialize(){
    mLimelightSubsystem->changePipeToAim();
}

void AimingVisionCommand::End(bool interrupted){
    mLimelightSubsystem->changePipeToDrive();
}

bool AimingVisionCommand::IsFinished(){
    return false;
}