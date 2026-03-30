// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  (void)constants::vision::kSyncBytes[0];
  RegisterAutoCommands();
  m_chooser = pathplanner::AutoBuilder::buildAutoChooser();
  
  // Initialize all of your commands and subsystems here 

  // Grant default control of swerve drive to either bill (controller 0),
  // or test (controller 2), based on a constant
  switch (constants::swerve::swerve_drive::kActiveController) {
    case(constants::swerve::ActiveDriveController::OfficialDriver) :
      m_swerveDrive->SetDefaultCommand(commands::main::Drive(&m_bill, m_swerveDrive, m_driveState));
      break;
    case(constants::swerve::ActiveDriveController::TestControls) :
      m_swerveDrive->SetDefaultCommand(commands::main::Drive(&m_test, m_swerveDrive, m_driveState));
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




void RobotContainer::ConfigureDriverBindings() {
  // Bill controls

  // functional
  frc2::Trigger resetPoseButton([this](){ return m_bill.GetTouchpadButton(); });
  resetPoseButton.OnTrue(actions::ResetPose(m_swerveDrive));

  frc2::Trigger runKickerMotorForward([this]() { return m_test.GetTriangleButton(); });
  runKickerMotorForward.OnTrue(actions::indexer::kicker::RunKickerForward(m_kicker));
  runKickerMotorForward.OnFalse(actions::indexer::kicker::ResetKicker(m_kicker));

  // frc2::Trigger runIntakeRollerMotorForward([this]() { return m_bill.GetL1Button(); });
  // runIntakeRollerMotorForward.OnTrue(actions::intake::IntakePowerForward(m_intake).ToPtr());
  // runIntakeRollerMotorForward.OnFalse(actions::intake::ResetIntakePower(m_intake)); 

  // // global states / 
  frc2::Trigger slowSwervesButton([this]() { return m_bill.GetR1Button() || m_bill.GetR2Button(); });
  slowSwervesButton.OnTrue(actions::swerve::SlowDownSwerves(m_swerveDrive, &m_driveState, &m_bill).ToPtr()); 
  slowSwervesButton.OnFalse(actions::swerve::DefaultSwerveSpeed(m_swerveDrive, &m_driveState, &m_bill).ToPtr()); 

  frc2::Trigger shootFuelButton([this](){ return m_bill.GetR2Button(); });
  shootFuelButton.OnTrue(actions::robot::AdvancedShoot(m_turret, m_flywheel, m_kicker, m_loader, m_articulation, m_intake).ToPtr());
  shootFuelButton.OnFalse(actions::robot::AdvancedShootEnd(m_kicker, m_loader, m_articulation, m_flywheel, m_intake));



  // uncomment to slow swerves while driving and uncomment the line in 
  frc2::Trigger shootFuelDumpButton([this](){ return m_bill.GetR1Button(); });
  shootFuelDumpButton.OnTrue(actions::robot::AdvancedShootSlowSwerves(m_turret, m_flywheel, m_kicker, m_loader, m_articulation, m_intake, m_swerveDrive, &m_driveState, &m_bill).ToPtr());
  shootFuelDumpButton.OnFalse(actions::robot::AdvancedShootSlowSwervesEnd(m_kicker, m_loader, m_articulation, m_flywheel, m_intake, m_swerveDrive, &m_driveState, &m_bill));

  frc2::Trigger setAlignStateButton([this]() { return m_bill.GetL2Button();}); 
  setAlignStateButton.OnTrue(actions::robot::ShooterStateHoodAndInterpAuto(&m_shooterState));
  setAlignStateButton.OnFalse(actions::robot::ShooterStateAutomatic(&m_shooterState));

  // frc2::Trigger toggleIntakeStateButton([this]() { return m_bill.GetL1Button(); }); 
  // toggleIntakeStateButton.OnTrue(actions::intake::ToggleIntakeState(m_intake));

  frc2::Trigger extendIntakeButton([this]() { return m_bill.GetL1Button(); }); 
  extendIntakeButton.OnTrue(actions::intake::IntakeExtendState(m_intake).ToPtr());
  extendIntakeButton.OnFalse(actions::intake::IntakeRetractState(m_intake).ToPtr());
  
  // frc2::Trigger ejectFuelButton([this] { return m_bill.GetR1Button(); }); 
  // ejectFuelButton.OnTrue(actions::robot::EjectFuel(m_intake, m_loader, m_kicker));
  // ejectFuelButton.OnFalse(actions::robot::EjectFuelEnd(m_intake, m_loader, m_kicker));

  // frc2::Trigger incrementFlywheelVelocityButton([this]() { return m_bill.GetTriangleButton(); });
  // incrementFlywheelVelocityButton.OnTrue(actions::shooter::flywheel::IncrementFlywheelVelocity(m_flywheel)); 

  // frc2::Trigger decrementFlywheelVelocityButton([this]() { return m_bill.GetCrossButton(); });
  // decrementFlywheelVelocityButton.OnTrue(actions::shooter::flywheel::DecrementFlywheelVelocity(m_flywheel)); 

  // frc2::Trigger incrementArticulationButton([this]() { return m_bill.GetSquareButton(); });
  // incrementArticulationButton.OnTrue(actions::shooter::hood::IncrementArticulationAngle(m_articulation)); 

  // frc2::Trigger decrementArticulationButton([this]() { return m_bill.GetCircleButton(); }); 
  // decrementArticulationButton.OnTrue(actions::shooter::hood::DecrementArticulationAngle(m_articulation)); 
}

void RobotContainer::RegisterAutoCommands() {
  // ! Note: commands should have a maximum time/deadline to be safe
  
  // Start of Auto Events
  pathplanner::NamedCommands::registerCommand("PrintAutoMessage", frc2::InstantCommand([this]{
    for (int i = 0; i < 10; i++) {
      std::cout << "Auto started/ended\n"; }},{}).ToPtr());

  pathplanner::NamedCommands::registerCommand("Intake", actions::intake::IntakeExtendState(m_intake).ToPtr());
  pathplanner::NamedCommands::registerCommand("Retract Intake", actions::intake::IntakeRetractState(m_intake).ToPtr());
  pathplanner::NamedCommands::registerCommand("Auto Shoot", actions::robot::AdvancedShootSlowSwerves(m_turret, m_flywheel, m_kicker, m_loader, m_articulation, m_intake, m_swerveDrive, &m_driveState, &m_bill).ToPtr()); 
  pathplanner::NamedCommands::registerCommand("Auto Shoot End", actions::robot::AdvancedShootSlowSwervesEnd(m_kicker, m_loader, m_articulation, m_flywheel, m_intake, m_swerveDrive, &m_driveState, &m_bill)); 
  pathplanner::NamedCommands::registerCommand("Auto Shoot", actions::robot::AdvancedShoot(m_turret, m_flywheel, m_kicker, m_loader, m_articulation, m_intake).ToPtr());
  pathplanner::NamedCommands::registerCommand("Auto Shoot End", actions::robot::AdvancedShootEnd(m_kicker, m_loader, m_articulation, m_flywheel, m_intake));
  pathplanner::NamedCommands::registerCommand("Set Pose Left Trench", actions::SetLeftTrenchAutoPose(m_swerveDrive));
  pathplanner::NamedCommands::registerCommand("Set Pose Right Trench", actions::SetRightTrenchAutoPose(m_swerveDrive));
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
  m_swerveDrive->SetBrakeMode(constants::BrakeMode::Coast);
}

void RobotContainer::SetAllBrake() {
  m_swerveDrive->SetBrakeMode(constants::BrakeMode::Brake);
  
}

// motors will coast along

void RobotContainer::SetAllNormalBrakeMode() {
  m_swerveDrive->SetBrakeMode(constants::BrakeMode::Default);
 
}

void RobotContainer::SetDriveBrake() {
  m_swerveDrive->SetBrakeMode(constants::BrakeMode::Brake);
}

void RobotContainer::SetDriveCoast() {
  m_swerveDrive->SetBrakeMode(constants::BrakeMode::Coast);
} 

void RobotContainer::PrintActiveCommands() {
  frc::SmartDashboard::PutString("Current command/swerve", m_swerveDrive->GetCurrentCommand()->GetName());
}

void RobotContainer::ConfigureTestBindings() {

 // Flywheel
  // frc2::Trigger runFlywheelMotorForward([this]() { return m_test.GetTriangleButton(); });
  // runFlywheelMotorForward.OnTrue(actions::shooter::flywheel::FlywheelPowerForward(m_flywheel));
  // runFlywheelMotorForward.OnFalse(actions::shooter::flywheel::ResetFlywheelPower(m_flywheel));
  
  // frc2::Trigger runFlywheelMotorReverse([this]() { return m_test.GetCrossButton(); });
  // runFlywheelMotorReverse.OnTrue(actions::shooter::flywheel::FlywheelPowerReverse(m_flywheel));
  // runFlywheelMotorReverse.OnFalse(actions::shooter::flywheel::ResetFlywheelPower(m_flywheel)); 

  // frc2::Trigger runArticulationRotationMotorForward([this]() { return m_bill.GetSquareButton(); }); 
  // runArticulationRotationMotorForward.OnTrue(actions::shooter::hood::RunArticulationForward(m_articulation)); 
  // runArticulationRotationMotorForward.OnFalse(actions::shooter::hood::ResetArticulationPower(m_articulation)); 

  // frc2::Trigger runArticulationRotationMotorReset([this]() { return m_bill.GetSquareButton(); }); 
  // runArticulationRotationMotorReset.OnTrue(actions::shooter::hood::RunArticulationReverse(m_articulation)); 
  // runArticulationRotationMotorReset.OnFalse(actions::shooter::hood::ResetArticulationPower(m_articulation)); 

  // // Articulation PID
  // frc2::Trigger setArticulationAngle0([this]() { return m_bill.GetSquareButton(); });
  // setArticulationAngle0.OnTrue(actions::shooter::hood::Articulation0Degrees(m_articulation)); 

  // frc2::Trigger setArticulationAngle20([this]() { return m_bill.GetSquareButton(); });
  // setArticulationAngle20.OnTrue(actions::shooter::hood::Articulation20Degrees(m_articulation)); 

  // // Kicker
  // frc2::Trigger runKickerMotorForward([this]() { return m_test.GetTriangleButton(); });
  // runKickerMotorForward.OnTrue(actions::indexer::kicker::RunKickerForward(m_kicker));
  // runKickerMotorForward.OnFalse(actions::indexer::kicker::ResetKicker(m_kicker));

  // frc2::Trigger runKickerMotorReverse([this]() { return m_test.GetCrossButton(); });
  // runKickerMotorReverse.OnTrue(actions::indexer::kicker::RunKickerBackward(m_kicker));
  // runKickerMotorReverse.OnFalse(actions::indexer::kicker::ResetKicker(m_kicker)); 

  // // Loader (belly belt)
  // frc2::Trigger runLoaderMotorForward([this]() { return m_test.GetSquareButton(); });
  // runLoaderMotorForward.OnTrue(actions::indexer::loader::RunLoaderForward(m_loader));
  // runLoaderMotorForward.OnFalse(actions::indexer::loader::ResetLoader(m_loader));

  // frc2::Trigger runLoaderMotorReverse([this]() { return m_test.GetCircleButton(); });
  // runLoaderMotorReverse.OnTrue(actions::indexer::loader::RunLoaderBackward(m_loader));
  // runLoaderMotorReverse.OnFalse(actions::indexer::loader::ResetLoader(m_loader)); 

  // // Intake
  // frc2::Trigger runIntakeRollerMotorForward([this]() { return m_test.GetTriangleButton(); });
  // runIntakeRollerMotorForward.OnTrue(actions::intake::IntakePowerForward(m_intake).ToPtr());
  // runIntakeRollerMotorForward.OnFalse(actions::intake::ResetIntakePower(m_intake)); 

  // frc2::Trigger runIntakeRollerMotorReverse([this]() { return m_test.GetCrossButton(); });
  // runIntakeRollerMotorReverse.OnTrue(actions::intake::IntakePowerReverse(m_intake));
  // runIntakeRollerMotorReverse.OnFalse(actions::intake::ResetIntakePower(m_intake)); 
  // frc2::Trigger runIntakeRotationMotorPowerForward([this]() { return m_test.GetL1Button(); });
  // runIntakeRotationMotorPowerForward.OnTrue(actions::intake::IntakeRotationPowerForward(m_intake));
  // runIntakeRotationMotorPowerForward.OnFalse(actions::intake::ResetIntakeRotationPower(m_intake));

  // frc2::Trigger runIntakeRotationMotorPowerReverse([this]() { return m_test.GetR1Button(); });
  // runIntakeRotationMotorPowerReverse.OnTrue(actions::intake::IntakeRotationPowerReverse(m_intake));
  // runIntakeRotationMotorPowerReverse.OnFalse(actions::intake::ResetIntakeRotationPower(m_intake));

  // // Intake PID Buttons
  frc2::Trigger toggleIntakeStateButton([this]() { return m_test.GetL1Button(); }); 
  toggleIntakeStateButton.OnTrue(actions::intake::ToggleIntakeState(m_intake));
  

  // frc2::Trigger setIntakeRotationDown([this]() { return m_test.GetSquareButton(); });
  // setIntakeRotationDown.OnTrue(actions::intake::IntakeRetractState(m_intake));

  // frc2::Trigger setIntakeRotationMid([this]() { return m_test.GetTriangleButton(); });
  // setIntakeRotationMid.OnTrue(actions::intake::IntakeMidState(m_intake));

  // frc2::Trigger setTurret45Deg([this](
  // frc2::Trigger setIntakeRotationUp([this]() { return m_test.GetCrossButton(); });
  // setIntakeRotationUp.OnTrue(actions::intake::IntakeExtendState(m_intake));

  // frc2::Trigger incrementFlywheelVelocityButton([this]() { return m_test.GetTriangleButton(); });
  // incrementFlywheelVelocityButton.OnTrue(actions::shooter::flywheel::IncrementFlywheelVelocity(m_flywheel)); 

  // frc2::Trigger decrementFlywheelVelocityButton([this]() { return m_test.GetCrossButton(); });
  // decrementFlywheelVelocityButton.OnTrue(actions::shooter::flywheel::DecrementFlywheelVelocity(m_flywheel)); 

  // frc2::Trigger testFlywheelFixedVelocityButton([this]() { return m_test.GetSquareButton(); }); 
  // testFlywheelFixedVelocityButton.OnTrue(actions::shooter::flywheel::TestFlywheelFixedVelocity(m_flywheel)); 

  // frc2::Trigger incrementArticulationButton([this]() { return m_test.GetSquareButton(); });
  // incrementArticulationButton.OnTrue(actions::shooter::hood::IncrementArticulationAngle(m_articulation)); 

  // frc2::Trigger decrementArticulationButton([this]() { return m_test.GetCircleButton(); }); 
  // decrementArticulationButton.OnTrue(actions::shooter::hood::DecrementArticulationAngle(m_articulation));

  // // // Turret
  // frc2::Trigger runTurretRotationMotorPowerForward([this]() { return m_test.GetL1Button(); });
  // runTurretRotationMotorPowerForward.OnTrue(actions::shooter::turret::TurretPowerForward(m_turret));
  // runTurretRotationMotorPowerForward.OnFalse(actions::shooter::turret::ResetTurretPower(m_turret));

  // frc2::Trigger runTurretRotationMotorPowerReverse([this]() { return m_test.GetR1Button(); });
  // runTurretRotationMotorPowerReverse.OnTrue(actions::shooter::turret::TurretPowerReverse(m_turret));
  // runTurretRotationMotorPowerReverse.OnFalse(actions::shooter::turret::ResetTurretPower(m_turret));

  // // Turret PID Buttons
  // frc2::Trigger setTurret45Deg([this]() { return m_test.GetSquareButton(); });
  // setTurret45Deg.OnTrue(actions::shooter::turret::PointTurretLeft(m_turret));

  // frc2::Trigger setTurret90Deg([this]() { return m_test.GetTriangleButton(); });
  // setTurret90Deg.OnTrue(actions::shooter::turret::PointTurretStraight(m_turret));

  // frc2::Trigger setTurret135Deg([this]() { return m_test.GetCircleButton(); });
  // setTurret135Deg.OnTrue(actions::shooter::turret::PointTurretRight(m_turret));
  
  // frc2::Trigger setTurret243Deg([this]() { return m_test.GetCrossButton(); });
  // setTurret243Deg.OnTrue(actions::shooter::turret::PointTurretMax(m_turret));

  // // Climber

  frc2::Trigger runClimberUp([this]() { return m_test.GetTriangleButton(); });
  runClimberUp.OnTrue(actions::climber::PutClimberUp(m_climber));
  runClimberUp.OnFalse(actions::climber::ResetClimber(m_climber));

  frc2::Trigger runClimberDown([this]() { return m_test.GetCrossButton(); });
  runClimberDown.OnTrue(actions::climber::PutClimberDown(m_climber));
  runClimberDown.OnFalse(actions::climber::ResetClimber(m_climber));

}

subsystems::shooter::Turret* RobotContainer::GetShooterSubsystem() {
  return m_turret; 
}

subsystems::shooter::Hood* RobotContainer::GetHoodSubsystem() {
  return m_articulation; 
}

util::interpolator::Data* RobotContainer::GetInterpolatorData() {
  return m_interpolatorData; 
}

subsystems::swerve::Drive* RobotContainer::GetSwerveSubsystem() {
  return m_swerveDrive; 
}


subsystems::shooter::Flywheel* RobotContainer::GetFlywheelSubsystem() {
  return m_flywheel;
}
  
  
constants::ShooterState RobotContainer::GetShooterState() {
  return m_shooterState;
}

subsystems::swerve::State RobotContainer::GetDriveState() {
  return m_driveState; 
}

void RobotContainer::SetShooterState(constants::ShooterState shooterState) {
  m_shooterState = shooterState;
}

frc::PS5Controller* RobotContainer::GetBill() {
  return &m_bill;
}


















// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣠⣤⣤⣤⣤⣤⣄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⢀⣴⣶⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡴⠶⠛⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠉⠓⠶⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⣴⠏⠀⢸⣿⠻⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⡴⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠳⢦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⣸⠁⠀⠀⢸⣿⠀⢱⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⡾⠋⠀⠀⠀⢀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⣇⢀⠀⠀⢸⠟⠃⠀⢧⡀⠀⠀⠀⠀⠀⠀⢀⣼⠏⠀⠀⣠⣤⣶⣿⣟⢳⣶⣶⣤⣄⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠘⠿⣶⣴⣤⣀⡀⠀⠀⠱⣄⠀⠀⠀⠀⢠⡿⢋⡇⢀⣾⡿⠿⠛⠋⠉⠉⠉⠛⠻⠿⢿⡿⠗⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢧⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠉⠙⠻⢦⣄⠀⠈⠳⢦⣀⣰⠟⢁⣨⡵⠏⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⣶⣶⠶⣦⣤⣀⣨⡷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠐⠲
// ⠀⠀⠀⠀⠀⠀⠀⠀⠙⢳⣄⠀⠀⢤⣥⡼⠛⠛⠇⠀⠀⠀⠀⠀⣀⣀⣉⣛⡳⢤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠷⠿⢿⣇⣽⣦⠀⠀⠀⠀⠀⠀⠀⠀⣠⡞⠁⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⠹⣦⣀⢚⠁⠂⠲⢦⣤⣀⣀⣤⣶⠿⢻⣭⣭⡛⠻⣦⡽⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⠀⠀⠀⠉⠙⢿⣧⠀⠀⠀⠀⠀⣠⡼⠋⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⡏⠈⠀⠀⠀⠀⠀⠉⠛⠿⣽⣶⣿⠿⣿⣿⣀⣈⣷⡸⣆⡤⠤⠂⠀⠀⠠⠀⡀⠀⣺⣭⣥⡴⣦⣤⣅⠀⠀⠀⢸⡀⠀⣠⡴⠚⠉⠀⠀⠲⣄⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⣿⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠛⠋⠊⠉⠁⠈⢁⡀⠀⠀⠀⠀⠀⠀⠳⣼⣏⣿⣿⣿⣦⠈⠙⢷⡀⠀⠀⠉⠉⠀⠀⢀⣀⣤⠴⠶⠿⠿⠦
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⡿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠻⠿⢿⣛⠒⠒⢛⠃⠀⠀⠀⢀⡤⠞⠋⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⢰⡿⢁⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⠀⠀⠀⠀⣴⣏⠀⠀⣀⡖⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠁⠀⠀⠀⢰⣏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⣾⡇⢸⠀⠀⠀⠀⠀⠀⣰⣾⡿⠿⠿⠛⠓⠀⠀⢿⣿⣿⣿⣿⣿⣶⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠋⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⢠⣿⢡⠘⠀⠀⠀⠀⠀⢚⡏⠁⠀⠀⠀⠀⠀⠀⠀⠀⠈⠁⠙⠻⢿⣿⣯⡛⠻⢶⣶⣶⣶⣶⣶⣿⣷⣶⣶⣁⠀⠀⠀⠀⠀⠀⠀⠀⠙⡆⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⢸⣿⣸⡁⠀⠀⠀⠀⠀⣾⠁⢙⢶⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠛⠓⠒⠒⠛⠉⠉⠉⠉⠉⠁⠈⠙⠶⣾⣷⣆⠀⠀⠀⠀⢹⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⣾⣧⣿⠃⠀⠀⠀⠀⢸⡇⠀⠈⠁⠈⠙⠳⢦⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢿⣷⠀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⢠⣿⡿⠸⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⠲⠦⢤⣤⣄⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡤⠈⢿⡇⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⣀⣴⡾⢻⣿⠁⠀⠀⠀⠀⠀⠀⠘⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠙⠛⠒⠒⠲⠶⠶⠶⠶⠦⢤⣤⠤⠴⠒⠋⠉⠀⠀⠈⠃⠀⠀⡞⠀⠀⠀⠀⠀⠀⠀⠀
// ⣴⣾⠿⣋⣭⣴⣿⠃⠀⠀⠀⠀⠀⠀⠀⠀⢻⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⡇⠀⠀⠀⠀⠀⠀⠀⠀
// ⣫⠾⣋⣽⣿⣷⣿⠀⠀⠀⠙⢧⡀⠀⠀⠀⠀⠷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣆⠀⠀⠀⠀⠀⠀⠀
// ⠋⣼⣿⢿⣶⣿⡇⠀⠀⠀⢀⠈⢿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡮⡛⣦⣄⠀⠀⠀⠀
// ⣤⣿⣿⣾⣿⣿⡆⠀⠀⠀⠙⠄⠈⠻⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⢻⣷⢹⡎⣿⣷⣤⣀⣀
// ⣿⣿⡿⣿⣿⠤⣫⡀⠀⠀⠀⠀⠀⠀⠈⢷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⢹⣬⣿⡞⢻⡿⣿⣧⡻⢿
// ⣿⣿⣾⣿⡯⣠⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠙⣷⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⠃⢸⣿⣿⠦⡄⢹⣾⣿⣷⠙
// ⣿⣿⣿⣿⡿⠋⣿⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡴⢀⡼⠃⠀⢸⣿⡇⢠⣄⣘⣿⣷⡿⣷
// ⣿⣿⣿⣿⡷⠘⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⣋⣴⠟⠁⠀⣠⡿⣿⣇⣀⠉⣿⣿⡋⠱⣏
// ⣿⣿⣿⣿⣧⠸⢻⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⡴⣾⣾⠟⠁⠀⠀⣼⡏⣷⣿⣛⢻⣶⣿⠿⠿⣆⣟
// ⣿⣿⣿⣿⣿⡂⠀⣿⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⣽⡾⠟⠁⠀⠀⠀⣴⡟⢷⣿⣏⠛⢿⣿⣦⣄⡀⠛⠛
// ⣿⣿⣿⣿⣿⣧⠠⠈⢿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠰⠶⠶⣶⣿⡿⠛⠉⠀⠀⠀⠀⢀⣾⣿⢻⣿⠿⣿⡷⣾⣿⣍⣿⡇⢀⣴
// ⣿⣿⣿⣿⣿⣿⡆⠀⠸⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠋⠓⠀⠠⠄⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠴⠟⠉⠀⠀⠀⠀⠀⠀⣀⣼⣿⠟⠁⣾⣿⣧⣼⣏⢿⣯⡀⣽⠃⣾⠁
// ⣿⣿⣿⣿⣿⣿⣿⡄⠀⢹⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠈⠁⠐⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣤⣼⡿⠋⢀⣾⣇⠈⣿⣏⣻⡟⠉⣿⢋⣿⠛⣷


// ! Signs of a legend btw
// no matter how nice ethan might seem, when you least expect it he will slap you with a piece of chicken and eat you in a bucket
// ! dont delete it please :<(   (._.)