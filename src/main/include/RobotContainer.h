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

#include "commands/main/Drive.h"
#include "commands/util/SetFlywheelMotorPower.h"
#include "commands/util/SetShooterKickerPower.h"
#include "commands/util/SetShooterRotationPower.h"
#include "commands/util/SetTurretAngle.h"
#include "commands/util/SetIntakeRollerMotor.h"
#include "commands/util/SetArticulationAngle.h"
#include "commands/main/SetIntakeAngleAndPower.h"


#include "util/align/Drivetrain.h"
#include "util/interpolator/Linear.h"
#include "util/interpolator/Data.h"

#include "subsystems/vision/PoseEstimatorHelper.h"
#include "subsystems/shooter/Turret.h"
#include "subsystems/vision/PhotonVision.h"
#include "subsystems/intake/Intake.h"

#include "subsystems/shooter/Articulation.h"
#include "subsystems/indexer/Kicker.h"
#include "subsystems/shooter/Flywheel.h"
#include "subsystems/indexer/Loader.h"



// #include "subsystems/Limelight.h"

#include "constants/Controller.h"
#include "constants/Automatic.h"
#include "constants/swerve/SwerveDrive.h"
#include "constants/shooter/Turret.h"
#include "constants/shooter/Articulation.h"
#include "constants/indexer/Kicker.h"
#include "constants/Interpolator.h"
#include "constants/Vision.h"

#include "actions/shooter/Articulation.h"
#include "actions/shooter/Flywheel.h"
#include "actions/indexer/Kicker.h"
#include "actions/indexer/Loader.h"
#include "actions/shooter/Turret.h"
#include "actions/Autonomous.h"
#include "actions/Global.h"
#include "actions/Intake.h"
#include "actions/Interpolator.h"
#include "actions/Robot.h"
#include "actions/Swerve.h"


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
  constants::BrakeMode GetBrakeMode();
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

  subsystems::shooter::Turret* GetShooterSubsystem();  
  subsystems::shooter::Articulation* GetHoodSubsystem(); 
  util::interpolator::Data* GetInterpolatorData(); 
  subsystems::swerve::Drive* GetSwerveSubsystem(); 
  subsystems::shooter::Flywheel* GetFlywheelSubsystem();
  subsystems::swerve::State GetDriveState(); 

  static void SetShooterState(constants::ShooterState shooterState);
  static constants::ShooterState GetShooterState();


 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc::PS5Controller m_bill{constants::controller::kDriverControllerPort};
  frc::PS5Controller m_test{constants::controller::kTestControllerPort};
 

  // frc::GenericHID m_buttonBoard{constants::controller::kButtonBoardPort}; 
  // The robot's subsystems are defined here..p.
  subsystems::vision::PoseEstimatorHelper *m_poseEstimator = new subsystems::vision::PoseEstimatorHelper();
  subsystems::swerve::Drive *m_swerveDrive = new subsystems::swerve::Drive(m_poseEstimator);
  
  subsystems::indexer::Loader *m_loader = new subsystems::indexer::Loader(); 
  subsystems::indexer::Kicker *m_kicker = new subsystems::indexer::Kicker(); 
  subsystems::shooter::Articulation *m_articulation = new subsystems::shooter::Articulation(m_swerveDrive);

  subsystems::shooter::Turret *m_turret = new subsystems::shooter::Turret(m_swerveDrive);
  subsystems::shooter::Flywheel *m_flywheel = new subsystems::shooter::Flywheel(m_swerveDrive); 

  subsystems::vision::Photon *m_turretCamera = new subsystems::vision::Photon(m_poseEstimator, 
    constants::vision::kPhotonTurretFrontCamera, 
    constants::vision::kPhotonTurretFrontCameraXOffset, 
    constants::vision::kPhotonTurretFrontCameraYOffset, 
    frc::Rotation2d{constants::vision::kPhotonTurretFrontCameraYawOffset},
    frc::Rotation2d{constants::vision::kPhotonTurretFrontCameraPitchOffset},
    constants::vision::kPhotonTurretFrontCameraHeight
  );

  subsystems::intake::Intake *m_intake = new subsystems::intake::Intake();

  util::interpolator::Data *m_interpolatorData = new util::interpolator::Data(constants::interpolator::kInterpolatorDataFilePath); 
  
    // old m_constraints
  pathplanner::PathConstraints m_constraints = pathplanner::PathConstraints(
    0.85_mps, 1.5_mps_sq,
    270_deg_per_s, 360_deg_per_s_sq);


  frc::SendableChooser<frc2::Command*> m_chooser;

  subsystems::swerve::State m_driveState = subsystems::swerve::State::HeadingControl;

  inline static constants::ShooterState m_shooterState = constants::ShooterState::Automatic;


  bool m_nomNomMode = false; 

  void ConfigureDriverBindings();
  void ConfigureCoDriverBindings();
  void ConfigureTestBindings();
  void CreateAutoPaths();
  void RegisterAutoCommands();
};

