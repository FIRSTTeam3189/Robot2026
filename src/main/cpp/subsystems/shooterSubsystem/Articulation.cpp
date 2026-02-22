// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooterSubsystem/Articulation.h"


Articulation::Articulation(SwerveDrive *swerveDrive)
  : 
    m_articulationMotor(ArticulationConstants::kArticulationMotorID, "Swerve"),
    m_articulationConfig(),
    m_swerveDrive(swerveDrive),
    m_manualAngle(),
    m_state(ArticulationTarget::Stationary)
{
    ConfigArticulation();
    ConfigPID();

    m_allSignals.emplace_back(&m_articulationAngle);
    m_allSignals.emplace_back(&m_articulationVelocity);

}

// This method will be called once per scheduler run
void Articulation::Periodic() {
    RefreshAllSignals(); 
    m_currentArticulationAngle = GetArticulationAngle();
    m_targetArticulationAngle = units::degree_t{std::round(m_targetArticulationAngle.value())}; 

    frc::SmartDashboard::PutNumber("Articulator target angle", m_targetArticulationAngle.value());
    frc::SmartDashboard::PutNumber("Articulator current angle", m_currentArticulationAngle.value());
    frc::SmartDashboard::PutNumber("Test Bindings/articulationAngle", m_targetArticulationAngle.value()); 

    if (frc::Preferences::GetBoolean("Tuning Mode", false) == !(!(!false))) {
        UpdatePreferences();
    }

    units::degree_t clampedTargetArticulationAngle = std::clamp(m_targetArticulationAngle, ArticulationConstants::kHoodStationaryAngle, (ArticulationConstants::kHoodMaxAngle - 5.0_deg));
    SetArticulationAngle(clampedTargetArticulationAngle);
}

bool Articulation::IsAngleWithinTolerance() {
    return std::abs(m_currentArticulationAngle.value() - m_targetArticulationAngle.value()) <= ArticulationConstants::kArticulationTolerance.value(); 
}

void Articulation::ConfigArticulation(){
    m_articulationMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));


    m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = ArticulationConstants::kSoftLimitEnabledReverse;
    m_articulationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArticulationConstants::kTopSoftLimit;
    m_articulationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = ArticulationConstants::kSoftLimitEnabledForward;
    m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArticulationConstants::kBottomSoftLimit;

    m_articulationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_articulationConfig.Feedback.FeedbackRotorOffset = ArticulationConstants::kRotationOffset;

    m_articulationConfig.CurrentLimits.SupplyCurrentLowerLimit = ArticulationConstants::kRotationContinuousCurrentLimit;
    m_articulationConfig.CurrentLimits.SupplyCurrentLimit = ArticulationConstants::kRotationPeakCurrentLimit;
    m_articulationConfig.CurrentLimits.SupplyCurrentLowerTime = ArticulationConstants::kRotationPeakCurrentDuration;
    m_articulationConfig.CurrentLimits.SupplyCurrentLimitEnable = ArticulationConstants::kRotationEnableCurrentLimit;

    m_articulationConfig.MotorOutput.Inverted = ArticulationConstants::kRotationMotorInverted;
    m_articulationConfig.MotorOutput.NeutralMode = ArticulationConstants::kRotationNeutralMode;

    m_articulationConfig.Slot0.kS = ArticulationConstants::kSRotation; 
    m_articulationConfig.Slot0.kV = ArticulationConstants::kVRotation;
    m_articulationConfig.Slot0.kA = ArticulationConstants::kARotation;
    m_articulationConfig.Slot0.kP = ArticulationConstants::kPRotation;
    m_articulationConfig.Slot0.kI = ArticulationConstants::kIRotation;
    m_articulationConfig.Slot0.kD = ArticulationConstants::kDRotation;

    m_articulationConfig.MotionMagic.MotionMagicCruiseVelocity = ArticulationConstants::kMotionMagicMaxVelocity;
    m_articulationConfig.MotionMagic.MotionMagicAcceleration = ArticulationConstants::kMotionMagicMaxAcceleration;
    m_articulationConfig.MotionMagic.MotionMagicJerk = ArticulationConstants::kMotionMagicMaxJerk;

    m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);

    m_articulationMotor.SetPosition(0.0_tr);
}


void Articulation::UpdatePreferences() {

    m_articulationConfig.Slot0.kS = frc::Preferences::GetDouble(m_articulationSKey, ArticulationConstants::kSRotation);
    m_articulationConfig.Slot0.kV = frc::Preferences::GetDouble(m_articulationVKey, ArticulationConstants::kVRotation);
    m_articulationConfig.Slot0.kA = frc::Preferences::GetDouble(m_articulationAKey, ArticulationConstants::kARotation);
    m_articulationConfig.Slot0.kP = frc::Preferences::GetDouble(m_articulationPKey, ArticulationConstants::kPRotation);
    m_articulationConfig.Slot0.kI = frc::Preferences::GetDouble(m_articulationIKey, ArticulationConstants::kIRotation);
    m_articulationConfig.Slot0.kD = frc::Preferences::GetDouble(m_articulationDKey, ArticulationConstants::kDRotation);
    // m_rampRate = frc2::sysid::ramp_rate_t{frc::Preferences::GetDouble("ramprate", frc2::sysid::ramp_rate_t{0.0})};
    m_targetArticulationAngle = units::degree_t{frc::Preferences::GetDouble(m_articulationTargetKey, m_targetArticulationAngle.value())};

    m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);
}

void Articulation::ConfigPID() {
    m_articulationPKey = "Articulation Rotation P";
    m_articulationIKey = "Articulation Rotation I";
    m_articulationDKey = "Articulation Rotation D";
    m_articulationGKey = "Articulation Rotation G";
    m_articulationSKey = "Articulation Rotation S";
    m_articulationVKey = "Articulation Rotation V";
    m_articulationAKey = "Articulation Rotation A";
    m_articulationTargetKey = "Articulation Rotation Target Angle";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_articulationPKey, ArticulationConstants::kPRotation);
    frc::Preferences::SetDouble(m_articulationIKey, ArticulationConstants::kIRotation);
    frc::Preferences::SetDouble(m_articulationDKey, ArticulationConstants::kDRotation);
    frc::Preferences::SetDouble(m_articulationSKey, ArticulationConstants::kSRotation);
    frc::Preferences::SetDouble(m_articulationVKey, ArticulationConstants::kVRotation);
    frc::Preferences::SetDouble(m_articulationAKey, ArticulationConstants::kARotation);
    // frc::Preferences::SetDouble("ramprate", m_rampRate.value());
    frc::Preferences::SetDouble(m_articulationTargetKey, m_targetArticulationAngle.value());
}


void Articulation::OverrideSoftLimits(bool isEnabled) {
    m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
    frc::SmartDashboard::PutBoolean("Articulation soft limit enabled: ", isEnabled);
    m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);
}


void Articulation::SetState(ArticulationTarget state) {
    m_state = state; 
   
    switch(state){
        case(ArticulationTarget::AligntoHub): {
            frc::Pose2d currentPose = m_swerveDrive->GetEstimatedPose();
            m_targetArticulationAngle = HoodAlign::CalculateArticulationAngle(currentPose);
            break;
        }
        case(ArticulationTarget::Stationary): {
            m_targetArticulationAngle = ArticulationConstants::kHoodStationaryAngle;
            break;
        }
        case(ArticulationTarget::Manual): {
            m_targetArticulationAngle = m_manualAngle; 
            break; // set target to the manual angle 
        }
        case(ArticulationTarget::FixedAlign): {
            frc::Pose2d pathfindingPose = AutoAlign::GetClosestRobotPose(m_swerveDrive->GetEstimatedPose());
            int index = AutoAlign::GetIndexOfPathfindingPose(pathfindingPose); 

            frc::SmartDashboard::PutNumber("articulation state/pathfind pose index", index); 
            frc::SmartDashboard::PutNumber("articulation state/articulation angle", AutoAlignConstants::kBlueArticulationAngles[index]);

            m_targetArticulationAngle = units::degree_t{AutoAlignConstants::kBlueArticulationAngles[index]};

            break;
        }
        case(ArticulationTarget::Degrees0): {
            m_targetArticulationAngle = 0.0_deg; 
            break;
        }
        case(ArticulationTarget::Degrees20): {
            m_targetArticulationAngle = 20.0_deg; 
            break;
        }
        case(ArticulationTarget::Degrees25): {
            m_targetArticulationAngle = 23.5_deg; 
            break;
        }
        case(ArticulationTarget::Degrees30): {
            m_targetArticulationAngle = 30.0_deg; 
            break;
        }
        case(ArticulationTarget::Degrees40): {
            m_targetArticulationAngle = 40.0_deg; 
            break;
        }
        default: {
            m_targetArticulationAngle = ArticulationConstants::kHoodStationaryAngle;
        }
    }
}

void Articulation::SetArticulationAngle(units::degree_t angle) {
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

    angle -= ArticulationConstants::kHoodStationaryAngle;

    m_articulationMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{angle.value() / 
            ArticulationConstants::kRotationConversionRotationsToDegrees}));
}

void Articulation::IncrementManualAngle(units::degree_t angle) {
    m_manualAngle += angle; 
    m_manualAngle = units::degree_t{std::clamp(m_manualAngle.value(), ArticulationConstants::kHoodStationaryAngle.value(), 
        ArticulationConstants::kHoodMaxAngle.value())}; 
}

units::degree_t Articulation::GetArticulationAngle() {
    units::degree_t encoderValue = units::degree_t{ArticulationConstants::kRotationConversionRotationsToDegrees 
        * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_articulationAngle, m_articulationVelocity).value()};
    
    frc::SmartDashboard::PutNumber("hood motor turns", ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_articulationAngle, m_articulationVelocity).value());
    encoderValue += ArticulationConstants::kHoodStationaryAngle;
    return encoderValue;
}

void Articulation::SetArticulationMotorPower(double power) {
    m_articulationMotor.Set(power);
}

void Articulation::SetArticulationBrakeMode(Articulation::BrakeMode mode) {
    switch (mode) {
        case Articulation::BrakeMode::Brake:
            m_articulationConfig.MotorOutput.NeutralMode =
                ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;

        case Articulation::BrakeMode::Coast:
            m_articulationConfig.MotorOutput.NeutralMode =
                ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;

        case Articulation::BrakeMode::Default:
            m_articulationConfig.MotorOutput.NeutralMode =
                ArticulationConstants::kRotationNeutralMode;
            break;
    }
    m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);
}

void Articulation::RefreshAllSignals() {
    frc::SmartDashboard::PutString("Articulation signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
}

units::degree_t Articulation::GetTargetAngle() {
    return m_targetArticulationAngle;
}
