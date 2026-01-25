// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>
#include <frc/PS5Controller.h>
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "commands/mainCommands/Drive.h"
#include "commands/utilCommands/SetFlywheelMotorPower.h"
#include "commands/utilCommands/SetShooterLoaderPower.h"
#include "commands/utilCommands/SetShooterRotationPower.h"
#include "commands/utilCommands/SetTurretAngle.h"
#include "commands/utilCommands/SetIntakeRollerMotor.h"

#include "util/alignUtils/AutoAlign.h"
#include "util/PowerInterpolator.h"


#include "subsystems/PoseEstimatorHelper.h"
#include "subsystems/Shooter.h"
#include "subsystems/PhotonVision.h"
#include "subsystems/Intake.h"


// #include "subsystems/Limelight.h"

#include "Constants/OperatorConstants.h"
#include "Constants/AutoConstants.h"
#include "Constants/SwerveDriveConstants.h"
#include "Constants/ShooterConstants.h"
#include "Constants/VisionConstants.h"


#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  BrakeMode GetBrakeMode();
  // void Pathfindbranches(frc::Pose2D pose, std::shared_ptr<pathplanner::PathPlannerPath> scorepath, std::shared_ptr<pathplanner::PathPlannerPath> retreatpath, string level);
  void SetDrivePose(frc::Pose2d pose, bool justRotation);
  void SetAllCoast();
  void SetAllBrake();
  void SetDriveBrake();
  void SetDriveCoast();
  void SetElevatorCoast();
  void SetAlgaeCoast();
  void SetAllNormalBrakeMode();
  void PrintActiveCommands();

  Shooter* GetShooterSubsystem(); 

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc::PS5Controller m_bill{OperatorConstants::kDriverControllerPort};
  frc::PS5Controller m_test{OperatorConstants::kTestControllerPort};
 

  // frc::GenericHID m_buttonBoard{OperatorConstants::kButtonBoardPort}; 
  // The robot's subsystems are defined here..p.
  PoseEstimatorHelper *m_poseEstimator = new PoseEstimatorHelper();
  SwerveDrive *m_swerveDrive = new SwerveDrive(m_poseEstimator);


  PowerInterpolator *m_powerInterpolator = new PowerInterpolator(
    AutoAlignConstants::kPowerInterpolatorDistances, AutoAlignConstants::kPowerInterpolatorPowers); 
  

  Shooter *m_shooter = new Shooter(m_swerveDrive, m_powerInterpolator);
  PhotonVision *m_turrentCamera = new PhotonVision(m_poseEstimator, 
    VisionConstants::kPhotonCamera3, 
    VisionConstants::kPhotonTurretCameraXOffset, 
    VisionConstants::kPhotonTurretCameraYOffset, 
    frc::Rotation2d{VisionConstants::kPhotonTurretCameraYawOffset});

  Intake *m_intake = new Intake(); 
  
    // old m_constraints
  // pathplanner::PathConstraints m_constraints = pathplanner::PathConstraints(
  //   0.85_mps, 1.5_mps_sq,
  //   270_deg_per_s, 360_deg_per_s_sq);


  frc::SendableChooser<frc2::Command*> m_chooser;

  DriveState m_driveState = DriveState::HeadingControl;

  void ConfigureDriverBindings();
  void ConfigureCoDriverBindings();
  void ConfigureTestBindings();
  void CreateAutoPaths();
  void RegisterAutoCommands();
};

