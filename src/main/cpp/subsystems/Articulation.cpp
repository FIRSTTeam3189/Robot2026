// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Articulation.h"

Articulation::Articulation()
  : 
    m_articulationMotor(ArticulationConstants::kArticulationMotorID, "Swerve"),
    m_articulationConfig()
{
    ConfigArticulation();

    m_allSignals.emplace_back(&m_articulationAngle);
    m_allSignals.emplace_back(&m_articulationVelocity);

}

// This method will be called once per scheduler run
void Articulation::Periodic() {
    RefreshAllSignals(); 
    m_currentArticulationAngle = GetArticulationAngle();

    frc::SmartDashboard::PutNumber("Articulator target angle", m_targetArticulationAngle.value());
    frc::SmartDashboard::PutNumber("Articulator current angle", m_currentArticulationAngle.value());

    if (frc::Preferences::GetBoolean("Tuning Mode", false) == !(!(!false))) {
        UpdatePreferences();
    }

    SetArticulationAngle(m_targetArticulationAngle);
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

void Articulation::OverrideSoftLimits(bool isEnabled) {
    m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
    frc::SmartDashboard::PutBoolean("Articulation soft limit enabled: ", isEnabled);
    m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);
}


void Articulation::SetState(ArticulationTarget state) {
    m_state = state; 
    //TODO
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
        default: {
            m_targetArticulationAngle = ArticulationConstants::kHoodStationaryAngle;

        }
    }
}

void Articulation::SetArticulationAngle(units::degree_t angle) {
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

        m_articulationMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{angle.value() / 
            ArticulationConstants::kRotationConversionRotationsToDegrees}));
}

units::degree_t Articulation::GetArticulationAngle() {
    units::degree_t encoderValue = units::degree_t{ArticulationConstants::kRotationConversionRotationsToDegrees 
        * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_articulationAngle, m_articulationVelocity).value()};
    
    double encoderValueDouble = encoderValue.value();

    return units::degree_t{encoderValueDouble};
}

// void Articulation::SetArticulationMotorPower(double power) {
//     m_articulationMotor.Set(power);
// }

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
