// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>

void Robot::RobotInit() {
  // Stops the Diagnostic Server automatically starting up
  // Comment to start it again (-1 disables it while 0 enables it)
  c_SetPhoenixDiagnosticsStartTime(-1);

  // Creates a streaming camera server
  auto camera = frc::CameraServer::StartAutomaticCapture();
  camera.SetResolution(380, 240);
  camera.SetFPS(20);
  // Default to first USB flash drive or home/lvuser/logs/ if none available
  ctre::phoenix6::SignalLogger::SetPath("");
  ctre::phoenix6::SignalLogger::Start();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *f
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  frc::SmartDashboard::PutString("FLYWHEEL CURRENT COMMAND", m_container.GetFlywheelSubsystem()->GetCurrentCommand()->GetName());

  

  m_autoWinners = frc::DriverStation::GetGameSpecificMessage();
  m_timeLeft = frc::DriverStation::GetMatchTime(); 

  if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
    if (m_autoWinners == "B") {
      m_autoWon = true;
    } else if (m_autoWinners == "R") {
      m_autoWon = false;
    }
  } else {
    if (m_autoWinners == "R") {
      m_autoWon = true;
    } else if (m_autoWinners == "B") {
      m_autoWon = false;
    }
  }

  if (m_autoWon) {
    if (m_timeLeft.value() <= 30) {
      m_shouldScore = true;
    } else if (m_timeLeft.value() <= 55) {
      m_shouldScore = true;
    } else if (m_timeLeft.value() <= 80) {
      m_shouldScore = false;
    } else if (m_timeLeft.value() <= 105) {
      m_shouldScore = true;
    } else if (m_timeLeft.value() <= 130) {
      m_shouldScore = false;
    } else if (m_timeLeft.value() <= 160) {
      m_shouldScore = true;
    }
  } else if (m_autoWon == false) {
    if (m_timeLeft.value() <= 30) {
      m_shouldScore = true;
    } else if (m_timeLeft.value() <= 55) {
      m_shouldScore = false;
    } else if (m_timeLeft.value() <= 80) {
      m_shouldScore = true;
    } else if (m_timeLeft.value() <= 105) {
      m_shouldScore = false;
    } else if (m_timeLeft.value() <= 160) {
      m_shouldScore = true;
    }
  }


  frc::SmartDashboard::PutBoolean("Won auto?", m_autoWon);
  frc::SmartDashboard::PutBoolean("Should score?", m_shouldScore);

  frc::SmartDashboard::PutNumber("Current Drive State", static_cast<int>(m_container.GetDriveState())); 

  constants::ShooterState shooterState = RobotContainer::GetShooterState();
  frc::SmartDashboard::PutNumber("Current Battery Voltage", frc::RobotController::GetBatteryVoltage().value());
  frc::SmartDashboard::PutNumber("Curret Shooter State", static_cast<int>(shooterState));
  switch (shooterState) {
    case (constants::ShooterState::Automatic) : {
      RobotContainer::SetShooterState(constants::ShooterState::Automatic);
      m_container.GetFlywheelSubsystem()->SetState(constants::shooter::FlywheelState::AutoVelocity);
      m_container.GetShooterSubsystem()->SetState(constants::shooter::TurretTarget::AlignHub);
      break;
    }
    case (constants::ShooterState::Fixed) : {
      RobotContainer::SetShooterState(constants::ShooterState::Fixed);
      m_container.GetFlywheelSubsystem()->SetState(constants::shooter::FlywheelState::FixedVelocity);
      m_container.GetShooterSubsystem()->SetState(constants::shooter::TurretTarget::PointStraight);
      break;
    }
    case (constants::ShooterState::Reset) : {
      RobotContainer::SetShooterState(constants::ShooterState::Reset);
      m_container.GetFlywheelSubsystem()->SetState(constants::shooter::FlywheelState::ZeroVelocity);
      m_container.GetShooterSubsystem()->SetState(constants::shooter::TurretTarget::Point0Deg);
      break;
    }
    case (constants::ShooterState::Manual) : {
      RobotContainer::SetShooterState(constants::ShooterState::Manual);
      m_container.GetFlywheelSubsystem()->SetState(constants::shooter::FlywheelState::ManualVelocity);
      m_container.GetShooterSubsystem()->SetState(constants::shooter::TurretTarget::AlignHub);
      break;
    }
    case (constants::ShooterState::HoodAndInterpAuto) : {
      RobotContainer::SetShooterState(constants::ShooterState::HoodAndInterpAuto);
      m_container.GetShooterSubsystem()->SetState(constants::shooter::TurretTarget::PointStraight);
      m_container.GetFlywheelSubsystem()->SetState(constants::shooter::FlywheelState::AutoVelocity);
      break;
    }
  }
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  m_container.SetAllBrake();
  // Safety in case we forget to save 
  m_container.GetInterpolatorData()->SaveToFile(); 
  // m_container.SetElevatorCoast();
  // m_container.SetDriveCoast();
  // m_container.SetAlgaeCoast();
  // m_container.SetDriveBrake();
}

void Robot::DisabledPeriodic() {
  // if (!m_epochSet) {
  //   if (frc::Timer::GetTimestamp() > 5.0_s) {
  //     m_container.SetVisionEpochOffset();
  //     m_epochSet = true;
  //   }
  // }
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // m_container.SetAllBrake();
  m_autonomousCommand = m_container.GetAutonomousCommand();
  // if (frc::DriverStation::GetAlliance()) {
  //   if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue) {
  //     m_container.SetDrivePose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}, true);
  //   } else {
  //     m_container.SetDrivePose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{180.0_deg}}, true);
  //   }
  // }

  // frc::SmartDashboard::PutBoolean("Auto command interrupted", false);

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {

  // m_container.PrintActiveCommands();
  
  // frc::SmartDashboard::PutBoolean("Auto command/is finished", m_autonomousCommand->IsFinished()); 
}

void Robot::TeleopInit() {
  m_container.SetAllBrake();
   if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
  }
  // m_container.Getshooter()->SetState(constants::shooter::TurretTarget::AlignHub); 
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // m_container.PrintActiveCommands();

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  (void)constants::vision::kSyncBytes[0];
  return frc::StartRobot<Robot>();
}
#endif
