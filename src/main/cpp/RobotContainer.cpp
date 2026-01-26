// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  (void)VisionConstants::kSyncBytes[0];
  RegisterAutoCommands();
  m_chooser = pathplanner::AutoBuilder::buildAutoChooser();
  
  // Initialize all of your commands and subsystems here 

  // Grant default control of swerve drive to either bill (controller 0),
  // or test (controller 2), based on a constant
  switch (SwerveDriveConstants::kActiveController) {
    case(ActiveDriveController::OfficialDriver) :
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
      break;
    case(ActiveDriveController::TestControls) :
      m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
      break;
    default :
      break;
 
  }


  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  
  // Configure the button bindings
  ConfigureDriverBindings();
  ConfigureTestBindings();
  CreateAutoPaths();
}

// void RobotContainer::Pathfindbranches(frc::Pose2D pose, std::shared_ptr<pathplanner::PathPlannerPath> scorepath, std::shared_ptr<pathplanner::PathPlannerPath> retreatpath, string level) {
//     std::move(pathplanner::AutoBuilder::pathfindToPose(pose)).AndThen(
//     std::move(std::move(pathplanner::NamedCommands::getCommand(level))).WithDeadline(std::move(pathplanner::AutoBuilder::followPath(scorepath)))).AndThen(
//     std::move(frc2::WaitCommand(1.0_s)).ToPtr()).AndThen(
//     std::move(pathplanner::NamedCommands::getCommand("ScoreCoral")).WithDeadline(std::move(frc2::WaitCommand(1.0_s)).ToPtr())).AndThen(
//     std::move(pathplanner::NamedCommands::getCommand("ResetElevator")).WithDeadline(std::move(pathplanner::AutoBuilder::followPath(m_retreatpath))))
// }



void RobotContainer::ConfigureDriverBindings() {
  // Bill controls

  frc2::Trigger resetPoseButton([this](){ return m_bill.GetTouchpadButton(); });
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetAutoPose(AutoAlignConstants::kBlueHubPose, false);
      else
        m_swerveDrive->SetAutoPose(AutoAlignConstants::kRedHubPose, false);
    }
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger applyTurretPowerForward([this](){ return m_bill.GetR1Button(); });
  applyTurretPowerForward.OnTrue(SetShooterRotationPower(m_turret, 0.2).ToPtr());
  applyTurretPowerForward.OnFalse(SetShooterRotationPower(m_turret, 0.0).ToPtr());

  frc2::Trigger applyTurretPowerReverse([this](){ return m_bill.GetL1Button(); });
  applyTurretPowerReverse.OnTrue(SetShooterRotationPower(m_turret, -0.2).ToPtr());
  applyTurretPowerReverse.OnFalse(SetShooterRotationPower(m_turret, 0.0).ToPtr());

  frc2::Trigger alignToHubButton([this](){ return m_bill.GetPOV() == 0; });
  alignToHubButton.OnTrue(frc2::ParallelCommandGroup(
    SetTurretAngle(m_turret, TurretTarget::AlignHub),
    SetArticulationAngle(m_articulation, ArticulationTarget::AligntoHub)
  ).ToPtr());
  // alignToHubButton.OnFalse(SetTurretAngle(m_turret, TurretTarget::PointStraight).ToPtr());

  // frc2::Trigger pointStraightButton([this](){ return m_bill.GetR2Button(); });
  // pointStraightButton.OnTrue(SetTurretAngle(m_turret, TurretTarget::PointStraight).ToPtr());

  frc2::Trigger pointLeftButton([this](){ return m_bill.GetCircleButton(); });
  pointLeftButton.OnTrue(SetTurretAngle(m_turret, TurretTarget::PointFerryLeft).ToPtr());

  frc2::Trigger pointRightButton([this](){ return m_bill.GetCrossButton(); });
  pointRightButton.OnTrue(SetTurretAngle(m_turret, TurretTarget::PointFerryRight).ToPtr());



  // frc2::Trigger slowDownButton([this](){ return m_bill.GetL2Button(); });
  // slowDownButton.OnTrue(frc2::InstantCommand([this]{
  //   m_driveState = DriveState::SlowDownSwerves;
  //   m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
  // },{m_swerveDrive}).ToPtr());
  // slowDownButton.OnFalse(frc2::InstantCommand([this]{
  //   m_driveState = DriveState::HeadingControl;
  //   m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
  // },{m_swerveDrive}).ToPtr());


  // frc2::Trigger runIntakeRollerForward([this]() {return m_bill.GetL2Button(); });
  // runIntakeRollerForward.OnTrue(SetIntakeRollerMotor(m_intake, 0.4).ToPtr()); 
  // runIntakeRollerForward.OnFalse(SetIntakeRollerMotor(m_intake, 0.0).ToPtr()); 

  // frc2::Trigger runIntakeRollerBackward([this](){ return m_bill.GetR2Button(); });
  // runIntakeRollerBackward.OnTrue(SetIntakeRollerMotor(m_intake, -0.4).ToPtr()); 
  // runIntakeRollerBackward.OnFalse(SetIntakeRollerMotor(m_intake, 0.0).ToPtr());

  frc2::Trigger extendAndRunIntake([this](){ return m_bill.GetR2Button(); });
  extendAndRunIntake.OnTrue(SetIntakeAngleAndPower(m_intake, 0.8, true).ToPtr());

  frc2::Trigger retractIntake([this](){ return m_bill.GetL2Button(); });
  retractIntake.OnTrue(SetIntakeAngleAndPower(m_intake, 0.0, false).ToPtr());  // passes in 0.0 but does nothing


  // Flywheel as intake 
  // frc2::Trigger runIntakeRollerForward([this]() {return m_bill.GetL2Button(); });
  // runIntakeRollerForward.OnTrue(SetFlywheelMotorPower(m_flywheel, 0.8).ToPtr()); 
  // runIntakeRollerForward.OnFalse(SetFlywheelMotorPower(m_flywheel, 0.0).ToPtr()); 

  // frc2::Trigger runIntakeRollerBackward([this](){ return m_bill.GetR2Button(); });
  // runIntakeRollerBackward.OnTrue(SetFlywheelMotorPower(m_flywheel, -0.8).ToPtr()); 
  // runIntakeRollerBackward.OnFalse(SetFlywheelMotorPower(m_flywheel, 0.0).ToPtr());   


  // SYSID

//  frc2::Trigger Quasistaticstuff([this](){ return m_bill.GetCrossButton(); });
//   Quasistaticstuff.OnTrue(m_turret->SysIdQuasistatic(frc2::sysid::Direction::kForward));


//   frc2::Trigger Quasistaticforward([this](){ return m_bill.GetCircleButton(); });
//   Quasistaticforward.OnTrue(m_turret->SysIdQuasistatic(frc2::sysid::Direction::kReverse));


  
//   frc2::Trigger DynamicStuff([this](){ return m_bill.GetTriangleButton(); });
//   DynamicStuff.OnTrue(m_turret->SysIdDynamic(frc2::sysid::Direction::kForward));


//   frc2::Trigger Dynamicforward([this](){ return m_bill.GetSquareButton(); });
//   Dynamicforward.OnTrue(m_turret->SysIdDynamic(frc2::sysid::Direction::kReverse));

//[this]{
  //     m_algaeIntake->SetRollerPower(AlgaeIntakeConstants::kRollerIntakePower);
  //   }, {m_algaeIntake}),


}

void RobotContainer::RegisterAutoCommands() {
  // ! Note: commands should have a maximum time/deadline to be safe

  // Start of Auto Events
  pathplanner::NamedCommands::registerCommand("PrintAutoMessage", frc2::InstantCommand([this]{
    for (int i = 0; i < 10; i++) {
      std::cout << "Auto started/ended\n"; }},{}).ToPtr());

} 

void RobotContainer::CreateAutoPaths() {
  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);

  // Logging callbacks for pathplanner -- current pose, target pose, and active path
  pathplanner::PathPlannerLogging::setLogCurrentPoseCallback([this](frc::Pose2d pose) {
    // Do whatever you want with the poses here
    m_poseEstimator->SetCurrentAutoPose(pose);
  });

  pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose) {
    // Do whatever you want with the poses here
    m_poseEstimator->SetTargetAutoPose(pose);
  });

  // Logging callback for the active path, this is sent as a vector of poses
  pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses) {
    // Do whatever you want with the poses here
      m_poseEstimator->SetActivePath(poses);
  });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  auto command = m_chooser.GetSelected();
  command->AddRequirements(m_swerveDrive);
  command->SetName("Auto command");
  return command;
}

void RobotContainer::SetDrivePose(frc::Pose2d pose, bool justRotation) {
  m_swerveDrive->SetPose(pose, justRotation);
}

void RobotContainer::SetAllCoast() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Coast);
}

void RobotContainer::SetAllBrake() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Brake);
  
}

// motors will coast along

void RobotContainer::SetAllNormalBrakeMode() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Default);
 
}

void RobotContainer::SetDriveBrake() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Brake);
}

void RobotContainer::SetDriveCoast() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Coast);
}



void RobotContainer::PrintActiveCommands() {
  // frc::SmartDashboard::PutString("Current command/swerve", m_swerveDrive->GetCurrentCommand()->GetName());
  // frc::SmartDashboard::PutString("Current command/coral elevator", m_coralElevator->GetCurrentCommand()->GetName());
  // frc::SmartDashboard::PutString("Current command/coral manipulator", m_coralManipulator->GetCurrentCommand()->GetName());
}

void RobotContainer::ConfigureTestBindings() {

  
}

Turret* RobotContainer::GetShooterSubsystem() {
  return m_turret; 
}


// no matter how nice ethan might seem, when you least expect it he will slap you with a piece of chicken and eat you in a bucket