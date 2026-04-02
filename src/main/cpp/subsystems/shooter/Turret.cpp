// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooter/Turret.h"

namespace subsystems::shooter {

    Turret::Turret(subsystems::swerve::Drive *swerveDrive)
      : 
        m_turretMotor(constants::shooter::turret::kShooterRotationID, "Swerve"),
        m_shooterConfig(),
        m_swerveDrive(swerveDrive),
        m_state(constants::shooter::TurretTarget::PointStraight),
        m_turretField(),
        m_targetField(),
        m_velocityField()
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
        frc::SmartDashboard::PutNumber("Turret State", static_cast<int>(m_state));
    
        // frc::SmartDashboard::PutNumber("Coral elevator enum state", static_cast<int>(m_state));
        frc::SmartDashboard::PutNumber("Turret target angle", m_targetRotationAngle.value());
        frc::SmartDashboard::PutNumber("Turret current angle", m_currentRotationAngle.value());
        frc::SmartDashboard::PutNumber("Turret motor turns", ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_rotationAngle, m_rotationVelocity).value());
    
        if (frc::Preferences::GetBoolean("Tuning Mode", false) == !(!(!false))) {
            UpdatePreferences();
        }
        
        if (m_state != constants::shooter::TurretTarget::Manual) {
            SetTurretAngle(m_targetRotationAngle);
        }

    }
    
    units::degree_t Turret::GetCurrentTargetAngle() {
        return m_targetRotationAngle;
    }
    
    void Turret::RefreshAllSignals() {
        frc::SmartDashboard::PutString("Turret signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
    }
    
    void Turret::ConfigTurret() {
        m_turretMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));
    
        m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants::shooter::turret::kSoftLimitEnabledForward;
        m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants::shooter::turret::kSoftLimitEnabledReverse;
        m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants::shooter::turret::kTopSoftLimit;
        m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants::shooter::turret::kBottomSoftLimit;
    
        m_shooterConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        m_shooterConfig.Feedback.FeedbackRotorOffset = constants::shooter::turret::kRotationOffset;
    
        m_shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = constants::shooter::turret::kRotationContinuousCurrentLimit;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimit = constants::shooter::turret::kRotationPeakCurrentLimit;
        m_shooterConfig.CurrentLimits.SupplyCurrentLowerTime = constants::shooter::turret::kRotationPeakCurrentDuration;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = constants::shooter::turret::kRotationEnableCurrentLimit;
    
        m_shooterConfig.MotorOutput.Inverted = constants::shooter::turret::kRotationMotorInverted;
        m_shooterConfig.MotorOutput.NeutralMode = constants::shooter::turret::kRotationNeutralMode;
    
        m_shooterConfig.Slot0.kS = constants::shooter::turret::kSRotation; 
        m_shooterConfig.Slot0.kV = constants::shooter::turret::kVRotation;
        m_shooterConfig.Slot0.kA = constants::shooter::turret::kARotation;
        m_shooterConfig.Slot0.kP = constants::shooter::turret::kPRotation;
        m_shooterConfig.Slot0.kI = constants::shooter::turret::kIRotation;
        m_shooterConfig.Slot0.kD = constants::shooter::turret::kDRotation;
    
        m_shooterConfig.MotionMagic.MotionMagicCruiseVelocity = constants::shooter::turret::kMotionMagicMaxVelocity;
        m_shooterConfig.MotionMagic.MotionMagicAcceleration = constants::shooter::turret::kMotionMagicMaxAcceleration;
        m_shooterConfig.MotionMagic.MotionMagicJerk = constants::shooter::turret::kMotionMagicMaxJerk;
            
             
        m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
        m_turretMotor.SetPosition(units::turn_t{constants::shooter::turret::kMinimumAngleNegative.value() / constants::shooter::turret::kRotationConversionRotationsToDegrees});
        SetState(constants::shooter::TurretTarget::PointNeg56Deg); 
    }

    void Turret::DisableReverseSoftLimit() {
        m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
    }

    void Turret::EnableReverseSoftLimit() {
        m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants::shooter::turret::kSoftLimitEnabledReverse;
        m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
    }

    void Turret::ZeroTurret() {
        m_turretMotor.SetPosition(units::turn_t{constants::shooter::turret::kMinimumAngleNegative.value() / constants::shooter::turret::kRotationConversionRotationsToDegrees});
        SetState(constants::shooter::TurretTarget::PointNeg56Deg); 
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
    
        frc::Preferences::SetDouble(m_rotationPKey, constants::shooter::turret::kPRotation);
        frc::Preferences::SetDouble(m_rotationIKey, constants::shooter::turret::kIRotation);
        frc::Preferences::SetDouble(m_rotationDKey, constants::shooter::turret::kDRotation);
        frc::Preferences::SetDouble(m_rotationSKey, constants::shooter::turret::kSRotation);
        frc::Preferences::SetDouble(m_rotationVKey, constants::shooter::turret::kVRotation);
        frc::Preferences::SetDouble(m_rotationAKey, constants::shooter::turret::kARotation);
        // frc::Preferences::SetDouble("ramprate", m_rampRate.value());
        frc::Preferences::SetDouble(m_rotationTargetKey, m_targetRotationAngle.value());
    }
    
    void Turret::UpdatePreferences() {
        m_shooterConfig.Slot0.kS = frc::Preferences::GetDouble(m_rotationSKey, constants::shooter::turret::kSRotation);
        m_shooterConfig.Slot0.kV = frc::Preferences::GetDouble(m_rotationVKey, constants::shooter::turret::kVRotation);
        m_shooterConfig.Slot0.kA = frc::Preferences::GetDouble(m_rotationAKey, constants::shooter::turret::kARotation);
        m_shooterConfig.Slot0.kP = frc::Preferences::GetDouble(m_rotationPKey, constants::shooter::turret::kPRotation);
        m_shooterConfig.Slot0.kI = frc::Preferences::GetDouble(m_rotationIKey, constants::shooter::turret::kIRotation);
        m_shooterConfig.Slot0.kD = frc::Preferences::GetDouble(m_rotationDKey, constants::shooter::turret::kDRotation);
        // m_rampRate = frc2::sysid::ramp_rate_t{frc::Preferences::GetDouble("ramprate", frc2::sysid::ramp_rate_t{0.0})};
        m_targetRotationAngle = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_targetRotationAngle.value())};
    
        m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
    }
    
    void Turret::OverrideSoftLimits(bool isEnabled) {
        m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
        frc::SmartDashboard::PutBoolean("Turret soft limit enabled: ", isEnabled);
        m_turretMotor.GetConfigurator().Apply(m_shooterConfig);
    }
    
    
    void Turret::SetState(constants::shooter::TurretTarget state) {
        m_state = state;
        // Figure out target angle then set rotation to this angle
    
        switch (state) {
            case (constants::shooter::TurretTarget::AlignHub): {
                frc::Pose2d currentPose = m_swerveDrive->GetEstimatedPose();
                frc::Pose2d targetPose = util::align::turret::CalculateTargetPose(currentPose);
    
                targetPose = util::align::turret::UpdateTargetWithVelocity(currentPose, targetPose, GetRobotVelocity(), GetRobotAngularVelocity(), m_targetField); 
    
                m_targetRotationAngle = units::degree_t{util::align::turret::GetTargetAngle(currentPose, targetPose, constants::auto_align::kTurretOffset,
                                                                                    m_turretField, m_velocityField)};
                break;
            }
            case (constants::shooter::TurretTarget::PointStraight): {
                m_targetRotationAngle = 90.0_deg;
                break;
            }
            case (constants::shooter::TurretTarget::Point45Deg): {
                m_targetRotationAngle = constants::auto_align::TurretAngleLeft;
                break;  
            }     
            case (constants::shooter::TurretTarget::Point135Deg): {
                m_targetRotationAngle = constants::auto_align::TurretAngleRight;
                break;
            }   
            case (constants::shooter::TurretTarget::Point0Deg): {
                m_targetRotationAngle = 0.0_deg;
                break;
            }
            case (constants::shooter::TurretTarget::PointNeg56Deg): {
                m_targetRotationAngle = constants::shooter::turret::kMinimumAngleNegative; 
                break; 
            }             
            case (constants::shooter::TurretTarget::PointMaxDeg): {
                m_targetRotationAngle = constants::shooter::turret::kMaximumAngle; 
                break; 
            }
            case (constants::shooter::TurretTarget::Manual): {
                break; 
            }
            default:
                m_targetRotationAngle = 90.0_deg;
                break;  
        }
        // Rotation will be set periodicially
    }
    
    void Turret::SetTurretAngle(units::degree_t angle) {
        ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};
        m_turretMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{angle.value() / 
            constants::shooter::turret::kRotationConversionRotationsToDegrees}));
    }
    
    units::degree_t Turret::GetTurretAngle() {
        units::degree_t encoderValue = units::degree_t{constants::shooter::turret::kRotationConversionRotationsToDegrees 
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
    
    
    void Turret::SetRotationBrakeMode(constants::BrakeMode mode) {
        switch (mode) {
            case constants::BrakeMode::Brake:
                m_shooterConfig.MotorOutput.NeutralMode =
                    ctre::phoenix6::signals::NeutralModeValue::Brake;
                break;
    
            case constants::BrakeMode::Coast:
                m_shooterConfig.MotorOutput.NeutralMode =
                    ctre::phoenix6::signals::NeutralModeValue::Coast;
                break;
    
            case constants::BrakeMode::Default:
                m_shooterConfig.MotorOutput.NeutralMode =
                    constants::shooter::turret::kRotationNeutralMode;
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

    units::angular_velocity::radians_per_second_t Turret::GetRobotAngularVelocity() {
        frc::ChassisSpeeds robotSpeeds = m_swerveDrive->GetRobotRelativeSpeeds();
    
        frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            robotSpeeds.vx, 
            robotSpeeds.vy, 
            robotSpeeds.omega, 
            m_swerveDrive->GetNormalizedYaw()
        );
    
        frc::SmartDashboard::PutNumber("Robot Velocities/omega", fieldRelativeSpeeds.omega.value());
        
        return fieldRelativeSpeeds.omega; 
    }

}

