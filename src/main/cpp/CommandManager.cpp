#include "CommandManager.h"


frc2::CommandPtr CommandManager::ResetPose(SwerveDrive *swerveDrive) {
    return frc2::InstantCommand([swerveDrive]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        swerveDrive->SetAutoPose(AutoAlignConstants::kBlueHubPose, false);
      else
        swerveDrive->SetAutoPose(AutoAlignConstants::kRedHubPose, false);
    }
  },{swerveDrive}).ToPtr();
}