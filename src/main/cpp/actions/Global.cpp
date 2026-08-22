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

frc2::CommandPtr actions::ToggleQuestnav(subsystems::vision::Questnav *questnav) {
    return frc2::InstantCommand([questnav]{
        questnav->ToggleQuestnav();
    }, {questnav}).ToPtr();
}

frc2::CommandPtr actions::ResetPose(subsystems::swerve::Drive *swerveDrive, subsystems::vision::Questnav *questNav) {
    return frc2::InstantCommand([swerveDrive, questNav]{
        if (!frc::DriverStation::GetAlliance()) {
            return;
        }

        // Single source of truth: whichever pose is picked here is used for
        // both the drivetrain reset and the Quest reset below.
        frc::Pose2d targetPose = (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
            ? constants::swerve::swerve_drive::kBlueResetPose
            : constants::swerve::swerve_drive::kRedResetPose;

        swerveDrive->SetAutoPose(targetPose, false);

        // SetPose() expects the Quest's own pose, not the robot's -- apply the
        // robot-center-to-Quest mounting offset before handing it off.
        frc::Transform3d questMountOffset{
            constants::vision::kQuestMountXOffset,
            constants::vision::kQuestMountYOffset,
            constants::vision::kQuestMountZOffset,
            frc::Rotation3d{constants::vision::kQuestMountRollOffset,
                             constants::vision::kQuestMountPitchOffset,
                             constants::vision::kQuestMountYawOffset}};
        frc::Pose3d questPose = frc::Pose3d{targetPose}.TransformBy(questMountOffset);
        questNav->SetPose(questPose);
    }, {swerveDrive}).ToPtr();
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