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
  resetPoseButton.OnTrue(CommandManager::ResetPose(m_swerveDrive));

  frc2::Trigger alignToHubButton([this](){ return m_bill.GetPOV() == 0; });
  alignToHubButton.OnTrue(CommandManager::AlignToHub(m_turret, m_articulation));
  // alignToHubButton.OnFalse(SetTurretAngle(m_turret, TurretTarget::PointStraight).ToPtr());

  // frc2::Trigger pointStraightButton([this](){ return m_bill.GetR2Button(); });
  // pointStraightButton.OnTrue(SetTurretAngle(m_turret, TurretTarget::PointStraight).ToPtr());

  frc2::Trigger pointLeftButton([this](){ return m_bill.GetCircleButton(); });
  pointLeftButton.OnTrue(CommandManager::PointTurretLeft(m_turret));

  frc2::Trigger pointRightButton([this](){ return m_bill.GetCrossButton(); });
  pointRightButton.OnTrue(CommandManager::PointTurretRight(m_turret));


  frc2::Trigger slowDownButton([this](){ return m_bill.GetL1Button(); });
  slowDownButton.OnTrue(CommandManager::SlowDownSwerves(m_swerveDrive, &m_driveState, &m_bill));
  slowDownButton.OnFalse(CommandManager::DefaultSwerveSpeed(m_swerveDrive, &m_driveState, &m_bill));


  frc2::Trigger extendOrRetractIntake([this](){ return m_bill.GetL2Button(); });
  extendOrRetractIntake.OnTrue(SetIntakeAngleAndPower(m_intake, m_climber, IntakeConstants::kRollerIntakePower).ToPtr()); // 0.8 when set, nothign when retract
  extendOrRetractIntake.OnFalse(frc2::InstantCommand([](){}, {m_intake}).ToPtr());  // idk bro
  


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

  frc2::Trigger incrementFlywheelPowerButton([this](){ return m_test.GetSquareButton(); });
  incrementFlywheelPowerButton.OnTrue(CommandManager::IncrementFlywheelPower(m_flywheel)); 
  frc2::Trigger decrementFlywheelPowerButton([this](){ return m_test.GetCircleButton(); });
  decrementFlywheelPowerButton.OnTrue(CommandManager::DecrementFlywheelPower(m_flywheel)); 



  frc2::Trigger climbButton([this]() { return m_test.GetTriangleButton(); });
  climbButton.OnTrue(CommandManager::Climb(m_climber, m_intake));
}

Turret* RobotContainer::GetShooterSubsystem() {
  return m_turret; 
}


// no matter how nice ethan might seem, when you least expect it he will slap you with a piece of chicken and eat you in a bucket