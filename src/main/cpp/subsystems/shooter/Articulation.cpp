// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/shooter/Articulation.h"

namespace subsystems::shooter { 

    Articulation::Articulation(subsystems::swerve::Drive *swerveDrive)
    : 
        m_articulationMotor(constants::shooter::articulation::kArticulationMotorID, "Swerve"),
        m_articulationConfig(),
        m_angleInterpolator(constants::auto_align::kPowerInterpolatorDistances, constants::auto_align::kAngleInterpolatorAngles),
        m_swerveDrive(swerveDrive),
        m_state(constants::shooter::ArticulationTarget::Stationary)
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

        units::degree_t clampedTargetArticulationAngle = std::clamp(m_targetArticulationAngle, constants::shooter::articulation::kHoodStationaryAngle, (constants::shooter::articulation::kHoodMaxAngle - 5.0_deg));
        SetArticulationAngle(clampedTargetArticulationAngle);
    }

    bool Articulation::IsAngleWithinTolerance() {
        return std::abs(m_currentArticulationAngle.value() - m_targetArticulationAngle.value()) <= constants::shooter::articulation::kArticulationTolerance.value(); 
    }

    void Articulation::ConfigArticulation(){
        m_articulationMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));


        m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants::shooter::articulation::kSoftLimitEnabledReverse;
        m_articulationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants::shooter::articulation::kTopSoftLimit;
        m_articulationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants::shooter::articulation::kSoftLimitEnabledForward;
        m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants::shooter::articulation::kBottomSoftLimit;

        m_articulationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        m_articulationConfig.Feedback.FeedbackRotorOffset = constants::shooter::articulation::kRotationOffset;

        m_articulationConfig.CurrentLimits.SupplyCurrentLowerLimit = constants::shooter::articulation::kRotationContinuousCurrentLimit;
        m_articulationConfig.CurrentLimits.SupplyCurrentLimit = constants::shooter::articulation::kRotationPeakCurrentLimit;
        m_articulationConfig.CurrentLimits.SupplyCurrentLowerTime = constants::shooter::articulation::kRotationPeakCurrentDuration;
        m_articulationConfig.CurrentLimits.SupplyCurrentLimitEnable = constants::shooter::articulation::kRotationEnableCurrentLimit;

        m_articulationConfig.MotorOutput.Inverted = constants::shooter::articulation::kRotationMotorInverted;
        m_articulationConfig.MotorOutput.NeutralMode = constants::shooter::articulation::kRotationNeutralMode;

        m_articulationConfig.Slot0.kS = constants::shooter::articulation::kSRotation; 
        m_articulationConfig.Slot0.kV = constants::shooter::articulation::kVRotation;
        m_articulationConfig.Slot0.kA = constants::shooter::articulation::kARotation;
        m_articulationConfig.Slot0.kP = constants::shooter::articulation::kPRotation;
        m_articulationConfig.Slot0.kI = constants::shooter::articulation::kIRotation;
        m_articulationConfig.Slot0.kD = constants::shooter::articulation::kDRotation;

        m_articulationConfig.MotionMagic.MotionMagicCruiseVelocity = constants::shooter::articulation::kMotionMagicMaxVelocity;
        m_articulationConfig.MotionMagic.MotionMagicAcceleration = constants::shooter::articulation::kMotionMagicMaxAcceleration;
        m_articulationConfig.MotionMagic.MotionMagicJerk = constants::shooter::articulation::kMotionMagicMaxJerk;

        m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);

        m_articulationMotor.SetPosition(0.0_tr);
        SetState(constants::shooter::ArticulationTarget::Degrees0);
    }


    void Articulation::UpdatePreferences() {

        m_articulationConfig.Slot0.kS = frc::Preferences::GetDouble(m_articulationSKey, constants::shooter::articulation::kSRotation);
        m_articulationConfig.Slot0.kV = frc::Preferences::GetDouble(m_articulationVKey, constants::shooter::articulation::kVRotation);
        m_articulationConfig.Slot0.kA = frc::Preferences::GetDouble(m_articulationAKey, constants::shooter::articulation::kARotation);
        m_articulationConfig.Slot0.kP = frc::Preferences::GetDouble(m_articulationPKey, constants::shooter::articulation::kPRotation);
        m_articulationConfig.Slot0.kI = frc::Preferences::GetDouble(m_articulationIKey, constants::shooter::articulation::kIRotation);
        m_articulationConfig.Slot0.kD = frc::Preferences::GetDouble(m_articulationDKey, constants::shooter::articulation::kDRotation);
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

        frc::Preferences::SetDouble(m_articulationPKey, constants::shooter::articulation::kPRotation);
        frc::Preferences::SetDouble(m_articulationIKey, constants::shooter::articulation::kIRotation);
        frc::Preferences::SetDouble(m_articulationDKey, constants::shooter::articulation::kDRotation);
        frc::Preferences::SetDouble(m_articulationSKey, constants::shooter::articulation::kSRotation);
        frc::Preferences::SetDouble(m_articulationVKey, constants::shooter::articulation::kVRotation);
        frc::Preferences::SetDouble(m_articulationAKey, constants::shooter::articulation::kARotation);
        // frc::Preferences::SetDouble("ramprate", m_rampRate.value());
        frc::Preferences::SetDouble(m_articulationTargetKey, m_targetArticulationAngle.value());
    }


    void Articulation::OverrideSoftLimits(bool isEnabled) {
        m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
        frc::SmartDashboard::PutBoolean("Articulation soft limit enabled: ", isEnabled);
        m_articulationMotor.GetConfigurator().Apply(m_articulationConfig);
    }


    void Articulation::SetState(constants::shooter::ArticulationTarget state) {
        m_state = state; 
    
        switch(state){
            case(constants::shooter::ArticulationTarget::AligntoHub): {
                frc::Pose2d currentPose = m_swerveDrive->GetEstimatedPose();
                double distanceToTarget = util::align::turret::GetDistanceToTarget(currentPose).value(); 
                frc::SmartDashboard::PutNumber("DISTANCE TO TARGET ARTICULATOR", distanceToTarget);
                m_targetArticulationAngle = units::degree_t{m_angleInterpolator.Predict(distanceToTarget)}; 
                
                // m_targetArticulationAngle = util::align::hood::CalculateArticulationAngle(currentPose);
                break;
            }
            case(constants::shooter::ArticulationTarget::Stationary): {
                m_targetArticulationAngle = constants::shooter::articulation::kHoodStationaryAngle;
                break;
            }
            case(constants::shooter::ArticulationTarget::Manual): {
                m_targetArticulationAngle = m_targetArticulationAngle; 
                break;
            }
            case(constants::shooter::ArticulationTarget::FixedAlign): {
                frc::Pose2d pathfindingPose = util::align::drivetrain::GetClosestRobotPose(m_swerveDrive->GetEstimatedPose());
                int index = util::align::drivetrain::GetIndexOfPathfindingPose(pathfindingPose); 

                frc::SmartDashboard::PutNumber("articulation state/pathfind pose index", index); 
                frc::SmartDashboard::PutNumber("articulation state/articulation angle", constants::auto_align::kBlueArticulationAngles[index]);

                m_targetArticulationAngle = units::degree_t{constants::auto_align::kBlueArticulationAngles[index]};

                break;
            }
            case(constants::shooter::ArticulationTarget::Degrees0): {
                m_targetArticulationAngle = 0.0_deg; 
                break;
            }
            case(constants::shooter::ArticulationTarget::Degrees20): {
                m_targetArticulationAngle = 20.0_deg; 
                break;
            }
            case(constants::shooter::ArticulationTarget::Degrees25): {
                m_targetArticulationAngle = 23.5_deg; 
                break;
            }
            case(constants::shooter::ArticulationTarget::Degrees30): {
                m_targetArticulationAngle = 30.0_deg; 
                break;
            }
            case(constants::shooter::ArticulationTarget::Degrees40): {
                m_targetArticulationAngle = 40.0_deg; 
                break;
            }
            default: {
                m_targetArticulationAngle = constants::shooter::articulation::kHoodStationaryAngle;
            }
        }
    }

    void Articulation::SetArticulationAngle(units::degree_t angle) {
        ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

        angle -= constants::shooter::articulation::kHoodStationaryAngle;

        m_articulationMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{angle.value() / 
                constants::shooter::articulation::kRotationConversionRotationsToDegrees}));
    }
    
    units::degree_t Articulation::GetArticulationAngle() {
        units::degree_t encoderValue = units::degree_t{constants::shooter::articulation::kRotationConversionRotationsToDegrees 
            * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_articulationAngle, m_articulationVelocity).value()};
        
        frc::SmartDashboard::PutNumber("hood motor turns", ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_articulationAngle, m_articulationVelocity).value());
        encoderValue += constants::shooter::articulation::kHoodStationaryAngle;
        return encoderValue;
    }

    void Articulation::SetArticulationMotorPower(double power) {
        m_articulationMotor.Set(power);
    }

    void Articulation::SetArticulationBrakeMode(constants::BrakeMode mode) {
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
                    constants::shooter::articulation::kRotationNeutralMode;
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

    void Articulation::SetTargetAngle(units::degree_t target) {
        m_targetArticulationAngle = target; 
    }

}
