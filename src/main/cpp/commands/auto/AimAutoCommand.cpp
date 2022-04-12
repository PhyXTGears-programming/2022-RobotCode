#include "commands/auto/AimAutoCommand.h"

AimAutoCommand::AimAutoCommand(double strafe, double forewards, double acceptableAngleError, limelight * limelight, VisionPipelineCommand * visionPipelineCommand, SwerveDrive * swerveDrive){
    AddRequirements(swerveDrive);
    mLimelight = limelight;
    mVisionPipelineCommand = visionPipelineCommand;
    mSwerveDrive = swerveDrive;
    mStrafe = strafe;
    mForewards = forewards;
    mAcceptableAngleError = acceptableAngleError;
}
void AimAutoCommand::Initialize(){
    mVisionPipelineCommand->Schedule();
}
void AimAutoCommand::Execute(){
    mSwerveDrive->setMotion(mForewards, mStrafe, mLimelight->PIDCalculate());
}
void AimAutoCommand::End(bool interrupted){
    mVisionPipelineCommand->Cancel();
    mLimelight->finishAim();
}
bool AimAutoCommand::IsFinished(){
    return !(std::abs(mLimelight->getAngle()) > mAcceptableAngleError);
}