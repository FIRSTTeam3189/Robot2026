#include "actions/Global.h"

frc2::CommandPtr actions::ResetPose(subsystems::swerve::Drive *swerveDrive) {
    return frc2::InstantCommand([swerveDrive]{
        if (frc::DriverStation::GetAlliance()) {
            if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
                swerveDrive->SetAutoPose(constants::swerve::swerve_drive::kBlueResetPose, false);
            else
                swerveDrive->SetAutoPose(constants::swerve::swerve_drive::kRedResetPose, false);
        }
    },{swerveDrive}).ToPtr();
}

frc2::CommandPtr actions::EmptyCommand() {
    return frc2::InstantCommand([](){}).ToPtr();
}