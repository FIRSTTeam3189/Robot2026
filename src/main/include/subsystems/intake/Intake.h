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

#include "constants/Intake.h"
#include <frc/DoubleSolenoid.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystemLoop.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "subsystems/intake/State.h"


namespace subsystems {
  namespace intake {
    class Intake : public frc2::SubsystemBase {
    public:
      Intake();
      void SetRollerPower(double power);
      void UpdatePreferences();
      void ConfigRollerMotor();

      void SetIntake(double power);
      void RetractIntake();

      void SetState(subsystems::intake::State state, double rollerPower);
      subsystems::intake::State GetState();

      /**
       * Will be called periodically whenever the CommandScheduler runs.
       */
      void Periodic() override;


    private:

      // rev::spark::SparkMax m_rollerMotor;
      ctre::phoenix6::hardware::TalonFX m_intakeRollerMotor;
      ctre::phoenix6::configs::TalonFXConfiguration m_intakeConfig {};

      frc::DoubleSolenoid m_intakeSolenoid;

      subsystems::intake::State m_state;
      double m_desiredPower;

    };
  };
};


