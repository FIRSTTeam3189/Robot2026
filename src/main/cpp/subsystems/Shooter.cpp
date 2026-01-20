// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(SwerveDrive* swerveDrive, PowerInterpolator* powerInterpolator)
  : 
    m_shooterTurretMotor(ShooterConstants::kShooterRotationID, "Swerve"),
    m_shooterFlywheelMotor(ShooterConstants::kShooterFlywheelID, "Swerve"),
    m_shooterConfig(),
    m_flywheelConfig(),
    m_swerveDrive(swerveDrive),
    m_state(TurretTarget::PointStraight),
    m_turretField(),
    m_targetField(),
    m_powerInterpolator(powerInterpolator)
{
    ConfigShooter();
    ConfigPID();
    
    m_allSignals.emplace_back(&m_rotationAngle);
    m_allSignals.emplace_back(&m_rotationVelocity);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
    RefreshAllSignals(); 
    m_currentRotationAngle = GetTurretAngle();

    // frc::SmartDashboard::PutNumber("Coral elevator enum state", static_cast<int>(m_state));
    frc::SmartDashboard::PutNumber("Turret target angle", m_targetRotationAngle.value());
    frc::SmartDashboard::PutNumber("Turret current angle", m_currentRotationAngle.value());

    if (frc::Preferences::GetBoolean("Tuning Mode", false) == !(!(!false))) {
        UpdatePreferences();
    }

    SetTurretAngle(m_targetRotationAngle);
    SetFlyWheelPowerTest(m_targetPower);
}


units::degree_t Shooter::GetCurrentTargetAngle() {
    return m_targetRotationAngle;
}



void Shooter::RefreshAllSignals() {
    frc::SmartDashboard::PutString("Turret signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
}



void Shooter::ConfigShooter() {
    m_shooterTurretMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = ShooterConstants::kSoftLimitEnabledForward;
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = ShooterConstants::kSoftLimitEnabledReverse;
    m_shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants::kTopSoftLimit;
    m_shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants::kBottomSoftLimit;

    m_shooterConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_shooterConfig.Feedback.FeedbackRotorOffset = ShooterConstants::kRotationOffset;

    m_shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants::kRotationContinuousCurrentLimit;
    m_shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants::kRotationPeakCurrentLimit;
    m_shooterConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants::kRotationPeakCurrentDuration;
    m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants::kRotationEnableCurrentLimit;

    m_shooterConfig.MotorOutput.Inverted = ShooterConstants::kRotationMotorInverted;
    m_shooterConfig.MotorOutput.NeutralMode = ShooterConstants::kRotationNeutralMode;

    m_shooterConfig.Slot0.kS = ShooterConstants::kSRotation; 
    m_shooterConfig.Slot0.kV = ShooterConstants::kVRotation;
    m_shooterConfig.Slot0.kA = ShooterConstants::kARotation;
    m_shooterConfig.Slot0.kP = ShooterConstants::kPRotation;
    m_shooterConfig.Slot0.kI = ShooterConstants::kIRotation;
    m_shooterConfig.Slot0.kD = ShooterConstants::kDRotation;

    m_shooterConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants::kMotionMagicMaxVelocity;
    m_shooterConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants::kMotionMagicMaxAcceleration;
    m_shooterConfig.MotionMagic.MotionMagicJerk = ShooterConstants::kMotionMagicMaxJerk;

    m_flywheelConfig.MotorOutput.Inverted = ShooterConstants::kRotationMotorInverted;
    m_flywheelConfig.MotorOutput.NeutralMode = ShooterConstants::kRotationNeutralMode;
    
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

    frc::Preferences::SetDouble(m_rotationPKey, ShooterConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, ShooterConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, ShooterConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationSKey, ShooterConstants::kSRotation);
    frc::Preferences::SetDouble(m_rotationVKey, ShooterConstants::kVRotation);
    frc::Preferences::SetDouble(m_rotationAKey, ShooterConstants::kARotation);
    // frc::Preferences::SetDouble("ramprate", m_rampRate.value());
    frc::Preferences::SetDouble(m_rotationTargetKey, m_targetRotationAngle.value());
}

void Shooter::UpdatePreferences() {
    m_shooterConfig.Slot0.kS = frc::Preferences::GetDouble(m_rotationSKey, ShooterConstants::kSRotation);
    m_shooterConfig.Slot0.kV = frc::Preferences::GetDouble(m_rotationVKey, ShooterConstants::kVRotation);
    m_shooterConfig.Slot0.kA = frc::Preferences::GetDouble(m_rotationAKey, ShooterConstants::kARotation);
    m_shooterConfig.Slot0.kP = frc::Preferences::GetDouble(m_rotationPKey, ShooterConstants::kPRotation);
    m_shooterConfig.Slot0.kI = frc::Preferences::GetDouble(m_rotationIKey, ShooterConstants::kIRotation);
    m_shooterConfig.Slot0.kD = frc::Preferences::GetDouble(m_rotationDKey, ShooterConstants::kDRotation);
    // m_rampRate = frc2::sysid::ramp_rate_t{frc::Preferences::GetDouble("ramprate", frc2::sysid::ramp_rate_t{0.0})};
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

    switch (state) {
        case (TurretTarget::AlignHub): {
            frc::Pose2d currentPose = m_swerveDrive->GetEstimatedPose();
            frc::Pose2d targetPose = CalculateTargetPose(currentPose);

            m_targetPower = m_powerInterpolator->GetPower(currentPose, targetPose); 
            targetPose = UpdateTargetWithVelocity(currentPose, targetPose); 

            m_targetRotationAngle = units::degree_t{this->GetTargetAngle(currentPose, targetPose, AutoAlignConstants::kTurretOffset)};
            break;
        }
        case (TurretTarget::PointStraight): {
            m_targetRotationAngle = 90.0_deg;
            m_targetPower = 0.0;
            break;
        }
        case (TurretTarget::PointFerryLeft): {
            m_targetRotationAngle = AutoAlignConstants::FerryAngleLeft;
            m_targetPower = 0.0;
            break;  
        }     
        case (TurretTarget::PointFerryRight): {
            m_targetRotationAngle = AutoAlignConstants::FerryAngleRight;
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

void Shooter::SetFlyWheelTargetPower(double power) {
    m_targetPower = power; 
}


void Shooter::SetFlyWheelPowerTest(double power) {
    frc::SmartDashboard::PutNumber("Power Interpolator/power", power); 
}

void Shooter::SetTurretAngle(units::degree_t angle) {
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

    if ((m_state == TurretTarget::PointStraight) && ((m_currentRotationAngle.value() < 90.05) && (m_currentRotationAngle.value() > 89.95))) {
        m_shooterTurretMotor.Set(0);
    } else {
        m_shooterTurretMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{angle.value() / 
            ShooterConstants::kRotationConversionRotationsToDegrees}));
    }
}

units::degree_t Shooter::GetTurretAngle() {
    units::degree_t encoderValue = units::degree_t{ShooterConstants::kRotationConversionRotationsToDegrees 
        * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_rotationAngle, m_rotationVelocity).value()};
    
    double encoderValueDouble = encoderValue.value();
    while (encoderValueDouble >= 360) {
        encoderValueDouble -= 360; 
    } 

    return units::degree_t{encoderValueDouble};
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
                ShooterConstants::kRotationNeutralMode;
            break;
    }

    m_shooterTurretMotor.GetConfigurator().Apply(m_shooterConfig);
}

frc::Pose2d Shooter::UpdateTargetWithVelocity(frc::Pose2d robotPose, frc::Pose2d targetPose) {
    Vector2d targetVec(targetPose); // Account for velocity 
    Vector2d robotVelocity = GetRobotSpeed();
    double robotSpeed = robotVelocity.Length(); 
    if (robotSpeed > AutoAlignConstants::kMinimumVelocityMultiplier * SwerveModuleConstants::kMaxSpeed.value()) {
        // Needs to multiply magnitude by a linear interpolation based on ball air time 
        targetVec = targetVec + robotVelocity * AutoAlignConstants::kVelocityMultiplier * -1; 
        return targetVec.ToPose2d(); 
    }
    return targetPose; 
}

frc::Pose2d Shooter::CalculateTargetPose(frc::Pose2d robotPose){
    frc::Pose2d targetPose(0.0_m, 0.0_m, frc::Rotation2d(0.0_deg));
    bool adjustTarget = false; 
    if (frc::DriverStation::GetAlliance()) {
        if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue) {

            if (robotPose.X() < AutoAlignConstants::kBlueHubPose.X()){
                targetPose = AutoAlignConstants::kBlueHubPose;
                adjustTarget = true; 
            } else {
                targetPose = frc::Pose2d(AutoAlignConstants::kMidpointOnFieldBlue, robotPose.Y(), frc::Rotation2d(0.0_deg)); // rot dont matter
            }

        } else {
            if (robotPose.X() > AutoAlignConstants::kRedHubPose.X()){
                targetPose = AutoAlignConstants::kRedHubPose;
                adjustTarget = true; 
            } else {
                targetPose = frc::Pose2d(AutoAlignConstants::kMidpointOnFieldRed, robotPose.Y(), frc::Rotation2d(0.0_deg)); // rot dont matter
            }
        }
    }


    return targetPose;
}

// returns angle from 180-0
double Shooter::GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset) {

    double robotRotation = robotPose.Rotation().Degrees().value(); 
    Vector2d robotHeading(robotRotation); // assuming robot Heading = turret default Heading 

    // Get the poses as vectors 
    Vector2d turretVec = Vector2d(robotPose) + turretOffset; 
    Vector2d targetVec(targetPose); 

    Vector2d turretToTarget = targetVec - turretVec; 

    double errorAngle = turretToTarget.SignedAngle(robotHeading);

    double targetAngle = std::clamp(errorAngle + 90, 0.0, 180.0);

    frc::Pose2d endTargetPose = frc::Pose2d{units::meter_t{turretToTarget.x} + robotPose.X(), units::meter_t{turretToTarget.y} + robotPose.Y(), 
        frc::Rotation2d(0.0_deg)};


    m_targetField.SetRobotPose(endTargetPose); 

    frc::SmartDashboard::PutData("End target pose rarrrwrwrwrrw", &m_targetField); 

    frc::Pose2d turretPoseForViz = frc::Pose2d{robotPose.X(), robotPose.Y(), frc::Rotation2d(units::degree_t(-targetAngle + 90 + robotRotation))};
    m_turretField.SetRobotPose(turretPoseForViz);
    
    frc::SmartDashboard::PutData("Turret Pose", &m_turretField);

    return targetAngle; // where 90 is the turret's default position  
}

Vector2d Shooter::GetRobotSpeed() {
    frc::ChassisSpeeds robotSpeeds = m_swerveDrive->GetRobotRelativeSpeeds(); 
    frc::SmartDashboard::PutNumber("Robot Velocities/vx", robotSpeeds.vx.value());
    frc::SmartDashboard::PutNumber("Robot Velocities/vy", robotSpeeds.vy.value());
    return {robotSpeeds.vx.value(), robotSpeeds.vy.value()}; 
}



