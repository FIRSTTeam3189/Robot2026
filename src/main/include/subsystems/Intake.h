#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Preferences.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>

#include "Constants/IntakeConstants.h"
#include <frc/DoubleSolenoid.h>

enum class IntakeState {ExtendIntake, RetractIntake};


class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void SetRollerPower(double power);
  void UpdatePreferences();
  void ConfigRollerMotor();

  void SetIntake(double power);
  void RetractIntake();

  void SetState(IntakeState state, double rollerPower);
  IntakeState GetState();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;


 private:

  rev::spark::SparkMax m_rollerMotor;
  rev::spark::SparkMaxConfig m_rollerConfig;

  frc::DoubleSolenoid m_intakeSolenoid;

  IntakeState m_state;

};


