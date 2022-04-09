#include "commands/auto/AimAutoCommand.h"

AimAutoCommand::AimAutoCommand(double strafe, double forewards, double acceptableAngleError, limelight * limelight, VisionPipelineCommand * visionPipelineCommand, SwerveDrive * swerveDrive){
    AddRequirements(swerveDrive);
    mLimelight = limelight;
    mVisionPipelineCommand = visionPipelineCommand;
    mSwerveDrive = swerveDrive;
    kStrafe = strafe;
    kForewards = forewards;
    kAcceptableAngleError = acceptableAngleError;
}
void AimAutoCommand::Initialize(){
    mVisionPipelineCommand->Schedule();
}
void AimAutoCommand::Execute(){
    mSwerveDrive->setMotion(kForewards, kStrafe, mLimelight->PIDCalculate());
}
void AimAutoCommand::End(bool interrupted){
    mVisionPipelineCommand->Cancel();
    mLimelight->finishAim();
}
bool AimAutoCommand::IsFinished(){
    if(std::abs(mLimelight->getAngle()) > kAcceptableAngleError){
        return false;
    } else {
        return true;
    }
}