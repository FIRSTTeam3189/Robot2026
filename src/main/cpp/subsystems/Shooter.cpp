// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter()
  : m_shooterTurretMotor(ShooterConstants::kShooterRotationID, "Swerve"),
    m_shooterFlywheelMotor(ShooterConstants::kShooterFlywheelID, "Swerve"),
    m_state(TurretTarget::DefaultAngle),
    m_shooterConfig(),
    m_flywheelConfig(),
    m_flywheelPlant(frc::LinearSystemId::IdentifyVelocitySystem<units::radian>(ShooterConstants::kFlywheelKv, ShooterConstants::kFlywheelKa)),
    m_observer(m_flywheelPlant,
      {3.0},   // How accurate we think our model is
      {0.01},  // How accurate we think our encoder data is
      20_ms),
    m_controller(m_flywheelPlant, {ShooterConstants::kFlywheelQelms}, {ShooterConstants::kFlywheelRelms}, 20_ms),
    m_loop(m_flywheelPlant, m_controller, m_observer, 12_V, 20_ms),
    m_voltReq(0_V),
    m_sysIdRoutine(
        frc2::sysid::Config{
            std::nullopt,  // !!! Use default ramp rate (1 V/s) !!! LOWER THE RAMP RATE FOR THE TURRET OR ELSE THE TURRETS GONNA EXPLODE
            4_V,           // Reduce dynamic step voltage to 4 to prevent brownout
            std::nullopt,  // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
            [](frc::sysid::State state)
            {
                ctre::phoenix6::SignalLogger::WriteString("state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::volt_t volts) { m_shooterTurretMotor.SetControl(m_voltReq.WithOutput(volts)); },
            [](auto) {},
            this
        }
    )
   // drive motor (loader), flywheel, turret rotation motor
{
    ConfigShooter();
}

frc2::CommandPtr Shooter::SysIdQuasistatic(frc2::sysid::Direction direction)
{
    return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Shooter::SysIdDynamic(frc2::sysid::Direction direction)
{
    return m_sysIdRoutine.Dynamic(direction);
}

void Shooter::ConfigShooter() {
    m_shooterTurretMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = ShooterConstants::kSoftLimitEnabledForward;
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = ShooterConstants::kSoftLimitEnabledReverse;
    m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants::kTopSoftLimit;
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants::kBottomSoftLimit;

    m_shooterConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_shooterConfig.Feedback.FeedbackRotorOffset = ShooterConstants::kExtensionOffset;

    m_shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants::kExtensionContinuousCurrentLimit;
    m_shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants::kExtensionPeakCurrentLimit;
    m_shooterConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants::kExtensionPeakCurrentDuration;
    m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants::kExtensionEnableCurrentLimit;

    m_shooterConfig.MotorOutput.Inverted = ShooterConstants::kExtensionMotorInverted;
    m_shooterConfig.MotorOutput.NeutralMode = ShooterConstants::kExtensionNeutralMode;

    m_shooterConfig.Slot0.kS = ShooterConstants::kSExtension; 
    m_shooterConfig.Slot0.kV = ShooterConstants::kVExtension;
    m_shooterConfig.Slot0.kA = ShooterConstants::kAExtension;
    m_shooterConfig.Slot0.kP = ShooterConstants::kPExtension;
    m_shooterConfig.Slot0.kI = ShooterConstants::kIExtension;
    m_shooterConfig.Slot0.kD = ShooterConstants::kDExtension;

    m_shooterConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants::kMotionMagicMaxVelocity;
    m_shooterConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants::kMotionMagicMaxAcceleration;
    m_shooterConfig.MotionMagic.MotionMagicJerk = ShooterConstants::kMotionMagicMaxJerk;

    m_flywheelConfig.MotorOutput.Inverted = ShooterConstants::kExtensionMotorInverted;
    m_flywheelConfig.MotorOutput.NeutralMode = ShooterConstants::kExtensionNeutralMode;
    
    m_shooterTurretMotor.GetConfigurator().Apply(m_shooterConfig);
    m_shooterFlywheelMotor.GetConfigurator().Apply(m_flywheelConfig);
    // m_shooterTurretMotor.SetPosition(0.0_tr);
}

void Shooter::ConfigPID() {
    m_rotationPKey = "Shooter Rotation P";
    m_rotationIKey = "Shooter Rotation I";
    m_rotationDKey = "Shooter Rotation D";
    m_rotationGKey = "Shooter Rotation G";
    m_rotationSKey = "Shooter Rotation S";
    m_rotationVKey = "Shooter Rotation V";
    m_rotationAKey = "Shooter Rotation A";
    m_rotationTargetKey = "Shooter Rotation Target Angle";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_rotationPKey, ShooterConstants::kPExtension);
    frc::Preferences::SetDouble(m_rotationIKey, ShooterConstants::kIExtension);
    frc::Preferences::SetDouble(m_rotationDKey, ShooterConstants::kDExtension);
    frc::Preferences::SetDouble(m_rotationSKey, ShooterConstants::kSExtension);
    frc::Preferences::SetDouble(m_rotationVKey, ShooterConstants::kVExtension);
    frc::Preferences::SetDouble(m_rotationAKey, ShooterConstants::kAExtension);
    frc::Preferences::SetDouble(m_rotationTargetKey, m_targetRotationAngle.value());
}

void Shooter::UpdatePreferences() {
    m_shooterConfig.Slot0.kS = frc::Preferences::GetDouble(m_rotationSKey, ShooterConstants::kSExtension);
    m_shooterConfig.Slot0.kV = frc::Preferences::GetDouble(m_rotationVKey, ShooterConstants::kVExtension);
    m_shooterConfig.Slot0.kA = frc::Preferences::GetDouble(m_rotationAKey, ShooterConstants::kAExtension);
    m_shooterConfig.Slot0.kP = frc::Preferences::GetDouble(m_rotationPKey, ShooterConstants::kPExtension);
    m_shooterConfig.Slot0.kI = frc::Preferences::GetDouble(m_rotationIKey, ShooterConstants::kIExtension);
    m_shooterConfig.Slot0.kD = frc::Preferences::GetDouble(m_rotationDKey, ShooterConstants::kDExtension);
    m_targetRotationAngle = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_targetRotationAngle.value())};

    m_shooterTurretMotor.GetConfigurator().Apply(m_shooterConfig);
}

void Shooter::OverrideSoftLimits(bool isEnabled) {
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
    frc::SmartDashboard::PutBoolean("Shooter soft limit enabled: ", isEnabled);
    m_shooterTurretMotor.GetConfigurator().Apply(m_shooterConfig);
}


void Shooter::SetState(TurretTarget state) {
    m_state = state;
    // Figure out target angle then set rotation to this angle
    //Uncomment ts out once the turret align algorithm is done
    switch (state) {
        case (TurretTarget::AlignHub):
//            m_targetRotationAngle = AutoAlignConstants::Hub;
            break;
        case (TurretTarget::AlignFerry):
 //           m_targetRotationAngle = AutoAlignConstants::Ferry;
            break;
    }
    // Extension will be set periodicially
}


void Shooter::SetTurretAngle(units::degree_t angle) {
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};


    if ((m_state == TurretTarget::DefaultAngle) && ((m_currentRotationAngle.value() < 90.05) && (m_currentRotationAngle.value() > 89.95))) {
        m_shooterTurretMotor.Set(0);
    } else {
    m_shooterTurretMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{angle.value() / ShooterConstants::kRotationConversionRotationsToDegrees}));
}
}

units::degree_t Shooter::GetTurretAngle() {
    return units::degree_t{ShooterConstants::kRotationConversionRotationsToDegrees * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_rotationAngle, m_rotationVelocity).value()};
}

void Shooter::SetShooterRotationMotorPower(double power) {
    m_shooterTurretMotor.Set(power);
}

void Shooter::SetShooterFlywheelMotorPower(double power) {
    m_shooterFlywheelMotor.Set(power);
}



void Shooter::SetRotationBrakeMode(Shooter::BrakeMode mode) {
    switch (mode) {
        case Shooter::BrakeMode::Brake:
            m_shooterConfig.MotorOutput.NeutralMode =
                ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;

        case Shooter::BrakeMode::Coast:
            m_shooterConfig.MotorOutput.NeutralMode =
                ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;

        case Shooter::BrakeMode::Default:
            m_shooterConfig.MotorOutput.NeutralMode =
                ShooterConstants::kExtensionNeutralMode;
            break;
    }

    m_shooterTurretMotor.GetConfigurator().Apply(m_shooterConfig);
}

 double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset) {
        double robotAngle = robotPose.Rotation().Degrees().value(); // Get robot angle
        Vector2d robotVec(robotPose); // Get robot's heading 
        robotVec = robotVec.Rotate(-robotAngle); // Rotate robot's heading to align with normal 

        Vector2d turretVec = robotVec + turretOffset; // Calculate turret's vector based on robot vector and offset
        Vector2d targetVec(targetPose); // Get target's position as a vector for calculation 

        Vector2d turretToTarget = (targetVec - turretVec).Normalize(); // Get vector from turret to target

        // Get the angle 
        return -turretToTarget.Angle() + 90; // + 90 Adjust from -90-90 to 0-180 
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
    // RefreshAllSignals(); signals not implemented yet
    m_currentRotationAngle = GetTurretAngle();

    // frc::SmartDashboard::PutNumber("Coral elevator enum state", static_cast<int>(m_state));
    frc::SmartDashboard::PutNumber("Turret target angle", m_targetRotationAngle.value());
    frc::SmartDashboard::PutNumber("Turret current angle", m_currentRotationAngle.value());

    if (frc::Preferences::GetBoolean("Tuning Mode", false) == !(!(!false))) {
        UpdatePreferences();
    }

    SetTurretAngle(m_targetRotationAngle);
}
