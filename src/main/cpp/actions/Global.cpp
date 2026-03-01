#include "actions/Global.h"



frc2::InstantCommand actions::SetRobotPose(subsystems::swerve::Drive *swerveDrive, frc::Pose2d blueRobotPose, frc::Pose2d redRobotPose) {
    return frc2::InstantCommand([swerveDrive, blueRobotPose, redRobotPose]{
        if (frc::DriverStation::GetAlliance()) {
            if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
                swerveDrive->SetAutoPose(blueRobotPose, false);
            else
                swerveDrive->SetAutoPose(redRobotPose, false);
        }
    },{swerveDrive});
}

frc2::CommandPtr actions::ResetPose(subsystems::swerve::Drive *swerveDrive) {
    return SetRobotPose(swerveDrive, constants::swerve::swerve_drive::kBlueResetPose, constants::swerve::swerve_drive::kRedResetPose).ToPtr(); 
}

frc2::CommandPtr actions::SetLeftTrenchAutoPose(subsystems::swerve::Drive *swerveDrive) {
    return frc2::SequentialCommandGroup(
        SetRobotPose(swerveDrive, constants::swerve::swerve_drive::kBlueLeftTrenchPose, constants::swerve::swerve_drive::kRedLeftTrenchPose),
        frc2::WaitCommand(0.5_s)
    ).ToPtr(); 
}

frc2::CommandPtr actions::SetRightTrenchAutoPose(subsystems::swerve::Drive *swerveDrive) {
    return frc2::SequentialCommandGroup(
        SetRobotPose(swerveDrive, constants::swerve::swerve_drive::kBlueRightTrenchPose, constants::swerve::swerve_drive::kRedRightTrenchPose),
        frc2::WaitCommand(0.5_s)
    ).ToPtr(); 
}

frc2::CommandPtr actions::EmptyCommand() {
    return frc2::InstantCommand([](){}).ToPtr();
}