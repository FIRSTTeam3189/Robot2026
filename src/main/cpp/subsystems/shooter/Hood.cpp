// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooter/Hood.h"

namespace subsystems::shooter { 

    Hood::Hood(subsystems::swerve::Drive *swerveDrive)
    : 
        m_articulationMotor(constants::shooter::hood::kArticulationMotorID, "Swerve"),
        m_articulationConfig(),
        m_angleInterpolator(constants::auto_align::kPowerInterpolatorDistances, constants::auto_align::kAngleInterpolatorAngles),
        m_swerveDrive(swerveDrive),
        m_state(constants::shooter::HoodTarget::Stationary)
    {
        ConfigArticulation();
        ConfigPID();

        m_allSignals.emplace_back(&m_articulationAngle);
        m_allSignals.emplace_back(&m_articulationVelocity);

    }

    // This method will be called once per scheduler run
    void Hood::Periodic() {
        RefreshAllSignals(); 
        m_currentArticulationAngle = GetArticulationAngle();
        m_targetArticulationAngle = units::degree_t{std::round(m_targetArticulationAngle.value())}; 

        frc::SmartDashboard::PutNumber("Articulator target angle", m_targetArticulationAngle.value());
        frc::SmartDashboard::PutNumber("Articulator current angle", m_currentArticulationAngle.value());
        frc::SmartDashboard::PutNumber("Test Bindings/articulationAngle", m_targetArticulationAngle.value()); 

        if (frc::Preferences::GetBoolean("Tuning Mode", false) == !(!(!false))) {
            UpdatePreferences();
        }

        units::degree_t clampedTargetArticulationAngle = std::clamp(m_targetArticulationAngle, constants::shooter::hood::kHoodStationaryAngle, (constants::shooter::hood::kHoodMaxAngle - 5.0_deg));
        SetHoodAngle(clampedTargetArticulationAngle);
    }

    bool Hood::IsAngleWithinTolerance() {
        return std::abs(m_currentArticulationAngle.value() - m_targetArticulationAngle.value()) <= constants::shooter::hood::kArticulationTolerance.value(); 
    }

    void Hood::ConfigArticulation(){
        m_articulationMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));


        m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants::shooter::hood::kSoftLimitEnabledReverse;
        m_articulationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants::shooter::hood::kTopSoftLimit;
        m_articulationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants::shooter::hood::kSoftLimitEnabledForward;
        m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants::shooter::hood::kBottomSoftLimit;

        m_articulationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        m_articulationConfig.Feedback.FeedbackRotorOffset = constants::shooter::hood::kRotationOffset;

        m_articulationConfig.CurrentLimits.SupplyCurrentLowerLimit = constants::shooter::hood::kRotationContinuousCurrentLimit;
        m_articulationConfig.CurrentLimits.SupplyCurrentLimit = constants::shooter::hood::kRotationPeakCurrentLimit;
        m_articulationConfig.CurrentLimits.SupplyCurrentLowerTime = constants::shooter::hood::kRotationPeakCurrentDuration;
        m_articulationConfig.CurrentLimits.SupplyCurrentLimitEnable = constants::shooter::hood::kRotationEnableCurrentLimit;

        m_articulationConfig.MotorOutput.Inverted = constants::shooter::hood::kRotationMotorInverted;
        m_articulationConfig.MotorOutput.NeutralMode = constants::shooter::hood::kRotationNeutralMode;

        m_articulationConfig.Slot0.kS = constants::shooter::hood::kSRotation; 
        m_articulationConfig.Slot0.kV = constants::shooter::hood::kVRotation;
        m_articulationConfig.Slot0.kA = constants::shooter::hood::kARotation;
        m_articulationConfig.Slot0.kP = constants::shooter::hood::kPRotation;
        m_articulationConfig.Slot0.kI = constants::shooter::hood::kIRotation;
        m_articulationConfig.Slot0.kD = constants::shooter::hood::kDRotation;

        m_articulationConfig.MotionMagic.MotionMagicCruiseVelocity = constants::shooter::hood::kMotionMagicMaxVelocity;
        m_articulationConfig.MotionMagic.MotionMagicAcceleration = constants::shooter::hood::kMotionMagicMaxAcceleration;
        m_articulationConfig.MotionMagic.MotionMagicJerk = constants::shooter::hood::kMotionMagicMaxJerk;

        m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);

        m_articulationMotor.SetPosition(0.0_tr);
        SetState(constants::shooter::HoodTarget::Degrees0);
    }


    void Hood::UpdatePreferences() {

        m_articulationConfig.Slot0.kS = frc::Preferences::GetDouble(m_articulationSKey, constants::shooter::hood::kSRotation);
        m_articulationConfig.Slot0.kV = frc::Preferences::GetDouble(m_articulationVKey, constants::shooter::hood::kVRotation);
        m_articulationConfig.Slot0.kA = frc::Preferences::GetDouble(m_articulationAKey, constants::shooter::hood::kARotation);
        m_articulationConfig.Slot0.kP = frc::Preferences::GetDouble(m_articulationPKey, constants::shooter::hood::kPRotation);
        m_articulationConfig.Slot0.kI = frc::Preferences::GetDouble(m_articulationIKey, constants::shooter::hood::kIRotation);
        m_articulationConfig.Slot0.kD = frc::Preferences::GetDouble(m_articulationDKey, constants::shooter::hood::kDRotation);
        // m_rampRate = frc2::sysid::ramp_rate_t{frc::Preferences::GetDouble("ramprate", frc2::sysid::ramp_rate_t{0.0})};
        m_targetArticulationAngle = units::degree_t{frc::Preferences::GetDouble(m_HoodTargetKey, m_targetArticulationAngle.value())};

        m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);
    }

    void Hood::ConfigPID() {
        m_articulationPKey = "Hood Rotation P";
        m_articulationIKey = "Hood Rotation I";
        m_articulationDKey = "Hood Rotation D";
        m_articulationGKey = "Hood Rotation G";
        m_articulationSKey = "Hood Rotation S";
        m_articulationVKey = "Hood Rotation V";
        m_articulationAKey = "Hood Rotation A";
        m_HoodTargetKey = "Hood Rotation Target Angle";
        //keys for PID and FF 

        frc::Preferences::SetDouble(m_articulationPKey, constants::shooter::hood::kPRotation);
        frc::Preferences::SetDouble(m_articulationIKey, constants::shooter::hood::kIRotation);
        frc::Preferences::SetDouble(m_articulationDKey, constants::shooter::hood::kDRotation);
        frc::Preferences::SetDouble(m_articulationSKey, constants::shooter::hood::kSRotation);
        frc::Preferences::SetDouble(m_articulationVKey, constants::shooter::hood::kVRotation);
        frc::Preferences::SetDouble(m_articulationAKey, constants::shooter::hood::kARotation);
        // frc::Preferences::SetDouble("ramprate", m_rampRate.value());
        frc::Preferences::SetDouble(m_HoodTargetKey, m_targetArticulationAngle.value());
    }


    void Hood::OverrideSoftLimits(bool isEnabled) {
        m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
        frc::SmartDashboard::PutBoolean("Hood soft limit enabled: ", isEnabled);
        m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);
    }


    void Hood::SetState(constants::shooter::HoodTarget state) {
        m_state = state; 
    
        switch(state){
            case(constants::shooter::HoodTarget::AligntoHub): {
                frc::Pose2d currentPose = m_swerveDrive->GetEstimatedPose();
                double distanceToTarget = util::align::turret::GetDistanceToTarget(currentPose).value(); 
                frc::SmartDashboard::PutNumber("DISTANCE TO TARGET ARTICULATOR", distanceToTarget);
                m_targetArticulationAngle = units::degree_t{m_angleInterpolator.Predict(distanceToTarget)}; 
                
                // m_targetArticulationAngle = util::align::hood::CalculateArticulationAngle(currentPose);
                break;
            }
            case(constants::shooter::HoodTarget::Stationary): {
                m_targetArticulationAngle = constants::shooter::hood::kHoodStationaryAngle;
                break;
            }
            case(constants::shooter::HoodTarget::Manual): {
                m_targetArticulationAngle = m_targetArticulationAngle; 
                break;
            }
            case(constants::shooter::HoodTarget::FixedAlign): {
                frc::Pose2d pathfindingPose = util::align::drivetrain::GetClosestRobotPose(m_swerveDrive->GetEstimatedPose());
                int index = util::align::drivetrain::GetIndexOfPathfindingPose(pathfindingPose); 

                frc::SmartDashboard::PutNumber("hood state/pathfind pose index", index); 
                frc::SmartDashboard::PutNumber("hood state/hood angle", constants::auto_align::kBlueArticulationAngles[index]);

                m_targetArticulationAngle = units::degree_t{constants::auto_align::kBlueArticulationAngles[index]};

                break;
            }
            case(constants::shooter::HoodTarget::Degrees0): {
                m_targetArticulationAngle = 0.0_deg; 
                break;
            }
            case(constants::shooter::HoodTarget::Degrees20): {
                m_targetArticulationAngle = 20.0_deg; 
                break;
            }
            case(constants::shooter::HoodTarget::Degrees25): {
                m_targetArticulationAngle = 23.5_deg; 
                break;
            }
            case(constants::shooter::HoodTarget::Degrees30): {
                m_targetArticulationAngle = 30.0_deg; 
                break;
            }
            case(constants::shooter::HoodTarget::Degrees40): {
                m_targetArticulationAngle = 40.0_deg; 
                break;
            }
            default: {
                m_targetArticulationAngle = constants::shooter::hood::kHoodStationaryAngle;
            }
        }
    }

    void Hood::SetHoodAngle(units::degree_t angle) {
        ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

        angle -= constants::shooter::hood::kHoodStationaryAngle;

        m_articulationMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{angle.value() / 
                constants::shooter::hood::kRotationConversionRotationsToDegrees}));
    }
    
    units::degree_t Hood::GetArticulationAngle() {
        units::degree_t encoderValue = units::degree_t{constants::shooter::hood::kRotationConversionRotationsToDegrees 
            * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_articulationAngle, m_articulationVelocity).value()};
        
        frc::SmartDashboard::PutNumber("Hood motor turns", ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_articulationAngle, m_articulationVelocity).value());
        encoderValue += constants::shooter::hood::kHoodStationaryAngle;
        return encoderValue;
    }

    void Hood::SetArticulationMotorPower(double power) {
        m_articulationMotor.Set(power);
    }

    void Hood::SetArticulationBrakeMode(constants::BrakeMode mode) {
        switch (mode) {
            case constants::BrakeMode::Brake:
                m_articulationConfig.MotorOutput.NeutralMode =
                    ctre::phoenix6::signals::NeutralModeValue::Brake;
                break;

            case constants::BrakeMode::Coast:
                m_articulationConfig.MotorOutput.NeutralMode =
                    ctre::phoenix6::signals::NeutralModeValue::Coast;
                break;

            case constants::BrakeMode::Default:
                m_articulationConfig.MotorOutput.NeutralMode =
                    constants::shooter::hood::kRotationNeutralMode;
                break;
        }
        m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);
    }

    void Hood::RefreshAllSignals() {
        frc::SmartDashboard::PutString("Hood signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
    }

    units::degree_t Hood::GetTargetAngle() {
        return m_targetArticulationAngle;
    }

    void Hood::SetTargetAngle(units::degree_t target) {
        m_targetArticulationAngle = target; 
    }

}
