// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooterSubsystem/Turret.h"

Turret::Turret(SwerveDrive* swerveDrive, Articulation* articulation)
  : 
    m_turretMotor(TurretConstants::kShooterRotationID, "Swerve"),
    m_shooterConfig(),
    m_swerveDrive(swerveDrive),
    m_state(TurretTarget::PointStraight),
    m_turretField(),
    m_targetField(),
    m_velocityField(),
    m_powerInterpolator(new LinearInterpolator(AutoAlignConstants::kPowerInterpolatorAngles, AutoAlignConstants::kPowerInterpolatorPowers)),
    m_articulation(articulation) 
{
    ConfigTurret();
    ConfigPID();
    
    m_allSignals.emplace_back(&m_rotationAngle);
    m_allSignals.emplace_back(&m_rotationVelocity);
}


// This method will be called once per scheduler run
void Turret::Periodic() {
    RefreshAllSignals(); 
    m_currentRotationAngle = GetTurretAngle();

    // frc::SmartDashboard::PutNumber("Coral elevator enum state", static_cast<int>(m_state));
    frc::SmartDashboard::PutNumber("Turret target angle", m_targetRotationAngle.value());
    frc::SmartDashboard::PutNumber("Turret current angle", m_currentRotationAngle.value());

    if (frc::Preferences::GetBoolean("Tuning Mode", false) == !(!(!false))) {
        UpdatePreferences();
    }

    SetTurretAngle(m_targetRotationAngle);
}

units::degree_t Turret::GetCurrentTargetAngle() {
    return m_targetRotationAngle;
}

void Turret::RefreshAllSignals() {
    frc::SmartDashboard::PutString("Turret signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
}

void Turret::ConfigTurret() {
    m_turretMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = TurretConstants::kSoftLimitEnabledForward;
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = TurretConstants::kSoftLimitEnabledReverse;
    m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants::kTopSoftLimit;
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants::kBottomSoftLimit;

    m_shooterConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_shooterConfig.Feedback.FeedbackRotorOffset = TurretConstants::kRotationOffset;

    m_shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants::kRotationContinuousCurrentLimit;
    m_shooterConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants::kRotationPeakCurrentLimit;
    m_shooterConfig.CurrentLimits.SupplyCurrentLowerTime = TurretConstants::kRotationPeakCurrentDuration;
    m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = TurretConstants::kRotationEnableCurrentLimit;

    m_shooterConfig.MotorOutput.Inverted = TurretConstants::kRotationMotorInverted;
    m_shooterConfig.MotorOutput.NeutralMode = TurretConstants::kRotationNeutralMode;

    m_shooterConfig.Slot0.kS = TurretConstants::kSRotation; 
    m_shooterConfig.Slot0.kV = TurretConstants::kVRotation;
    m_shooterConfig.Slot0.kA = TurretConstants::kARotation;
    m_shooterConfig.Slot0.kP = TurretConstants::kPRotation;
    m_shooterConfig.Slot0.kI = TurretConstants::kIRotation;
    m_shooterConfig.Slot0.kD = TurretConstants::kDRotation;

    m_shooterConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants::kMotionMagicMaxVelocity;
    m_shooterConfig.MotionMagic.MotionMagicAcceleration = TurretConstants::kMotionMagicMaxAcceleration;
    m_shooterConfig.MotionMagic.MotionMagicJerk = TurretConstants::kMotionMagicMaxJerk;
        
         
    m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
    // m_turretMotor.SetPosition(0.0_tr);
}


void Turret::ConfigPID() {
    m_rotationPKey = "Turret Rotation P";
    m_rotationIKey = "Turret Rotation I";
    m_rotationDKey = "Turret Rotation D";
    m_rotationGKey = "Turret Rotation G";
    m_rotationSKey = "Turret Rotation S";
    m_rotationVKey = "Turret Rotation V";
    m_rotationAKey = "Turret Rotation A";
    m_rotationTargetKey = "Turret Rotation Target Angle";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_rotationPKey, TurretConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, TurretConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, TurretConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationSKey, TurretConstants::kSRotation);
    frc::Preferences::SetDouble(m_rotationVKey, TurretConstants::kVRotation);
    frc::Preferences::SetDouble(m_rotationAKey, TurretConstants::kARotation);
    // frc::Preferences::SetDouble("ramprate", m_rampRate.value());
    frc::Preferences::SetDouble(m_rotationTargetKey, m_targetRotationAngle.value());
}

void Turret::UpdatePreferences() {
    m_shooterConfig.Slot0.kS = frc::Preferences::GetDouble(m_rotationSKey, TurretConstants::kSRotation);
    m_shooterConfig.Slot0.kV = frc::Preferences::GetDouble(m_rotationVKey, TurretConstants::kVRotation);
    m_shooterConfig.Slot0.kA = frc::Preferences::GetDouble(m_rotationAKey, TurretConstants::kARotation);
    m_shooterConfig.Slot0.kP = frc::Preferences::GetDouble(m_rotationPKey, TurretConstants::kPRotation);
    m_shooterConfig.Slot0.kI = frc::Preferences::GetDouble(m_rotationIKey, TurretConstants::kIRotation);
    m_shooterConfig.Slot0.kD = frc::Preferences::GetDouble(m_rotationDKey, TurretConstants::kDRotation);
    // m_rampRate = frc2::sysid::ramp_rate_t{frc::Preferences::GetDouble("ramprate", frc2::sysid::ramp_rate_t{0.0})};
    m_targetRotationAngle = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_targetRotationAngle.value())};

    m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
}

void Turret::OverrideSoftLimits(bool isEnabled) {
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
    frc::SmartDashboard::PutBoolean("Turret soft limit enabled: ", isEnabled);
    m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
}


void Turret::SetState(TurretTarget state) {
    m_state = state;
    // Figure out target angle then set rotation to this angle

    switch (state) {
        case (TurretTarget::AlignHub): {
            frc::Pose2d currentPose = m_swerveDrive->GetEstimatedPose();
            frc::Pose2d targetPose = TurretAlign::CalculateTargetPose(currentPose);

            double hoodAngle = m_articulation->GetTargetAngle().value(); 
            m_targetPower = m_powerInterpolator->Predict(hoodAngle); 

            frc::SmartDashboard::PutNumber("Power Interpolator/power", m_targetPower); 

            targetPose = TurretAlign::UpdateTargetWithVelocity(currentPose, targetPose, GetRobotVelocity(), m_targetField); 

            m_targetRotationAngle = units::degree_t{TurretAlign::GetTargetAngle(currentPose, targetPose, AutoAlignConstants::kTurretOffset,
                                                                                m_turretField, m_velocityField)};
            break;
        }
        case (TurretTarget::PointStraight): {
            m_targetRotationAngle = 90.0_deg;
            m_targetPower = 0.0;
            break;
        }
        case (TurretTarget::Point45Deg): {
            m_targetRotationAngle = AutoAlignConstants::TurretAngleLeft;
            m_targetPower = 0.0;
            break;  
        }     
        case (TurretTarget::Point135Deg): {
            m_targetRotationAngle = AutoAlignConstants::TurretAngleRight;
            m_targetPower = 0.0;
            break;
        }   
        default:
            m_targetRotationAngle = 90.0_deg;
            m_targetPower = 0.0;
            break;  
    }
    // Rotation will be set periodicially
}

void Turret::SetTurretAngle(units::degree_t angle) {
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

    if ((m_state == TurretTarget::PointStraight) && ((m_currentRotationAngle.value() < 90.05) && (m_currentRotationAngle.value() > 89.95))) {
        m_turretMotor.Set(0);
    } else {
        m_turretMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{angle.value() / 
            TurretConstants::kRotationConversionRotationsToDegrees}));
    }
}

units::degree_t Turret::GetTurretAngle() {
    units::degree_t encoderValue = units::degree_t{TurretConstants::kRotationConversionRotationsToDegrees 
        * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_rotationAngle, m_rotationVelocity).value()};
    
    double encoderValueDouble = encoderValue.value();
    while (encoderValueDouble >= 360) {
        encoderValueDouble -= 360; 
    } 

    return units::degree_t{encoderValueDouble};
}

void Turret::SetTurretRotationMotorPower(double power) {
    m_turretMotor.Set(power);
}


void Turret::SetRotationBrakeMode(Turret::BrakeMode mode) {
    switch (mode) {
        case Turret::BrakeMode::Brake:
            m_shooterConfig.MotorOutput.NeutralMode =
                ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;

        case Turret::BrakeMode::Coast:
            m_shooterConfig.MotorOutput.NeutralMode =
                ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;

        case Turret::BrakeMode::Default:
            m_shooterConfig.MotorOutput.NeutralMode =
                TurretConstants::kRotationNeutralMode;
            break;
    }

    m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
}



Vector2d Turret::GetRobotVelocity() {
    frc::ChassisSpeeds robotSpeeds = m_swerveDrive->GetRobotRelativeSpeeds();

    frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        robotSpeeds.vx, 
        robotSpeeds.vy, 
        robotSpeeds.omega, 
        m_swerveDrive->GetNormalizedYaw()
    );

    frc::SmartDashboard::PutNumber("Robot Velocities/vx", fieldRelativeSpeeds.vx.value());
    frc::SmartDashboard::PutNumber("Robot Velocities/vy", fieldRelativeSpeeds.vy.value());

    
    return {fieldRelativeSpeeds.vx.value(), fieldRelativeSpeeds.vy.value()}; 
}



Turret::~Turret() {
    delete m_powerInterpolator;
}