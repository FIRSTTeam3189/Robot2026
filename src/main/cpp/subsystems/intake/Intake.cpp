#include "subsystems/intake/Intake.h"

namespace subsystems::intake
{
    Intake::Intake() :

                       m_intakeRollerMotor(constants::intake::kRollerMotorID),
                       m_rollerConfig(),
                       m_intakeArticulationMotor(constants::intake::kArticulationMotorID, "Swerve"),
                       m_articulationConfig(),
                       m_state(State::RetractIntake)

    {
        ConfigIntakeMotors();
    }

    void Intake::ConfigIntakeMotors()
    {
        m_intakeRollerMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));
        m_intakeArticulationMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

        m_rollerConfig.MotorOutput.Inverted = constants::intake::kRollerMotorInverted;
        m_rollerConfig.MotorOutput.NeutralMode = constants::intake::kRollerMotorNeutralMode;

        // m_articulationConfig.CurrentLimits.SupplyCurrentLimitEnable = constants::intake::kRotationEnableCurrentlimit;
        // m_articulationConfig.CurrentLimits.SupplyCurrentLimit = constants::intake::kRotationContinuousCurrentLimit;

        m_articulationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants::intake::kSoftLimitEnabledForward;
        m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants::intake::kSoftLimitEnabledReverse;
        m_articulationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants::intake::kTopSoftLimit;
        m_articulationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants::intake::kBottomSoftLimit;

        m_articulationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;

        m_articulationConfig.CurrentLimits.SupplyCurrentLowerLimit = constants::intake::kRotationContinuousCurrentLimit;
        m_articulationConfig.CurrentLimits.SupplyCurrentLimit = constants::intake::kRotationPeakCurrentLimit;
        m_articulationConfig.CurrentLimits.SupplyCurrentLowerTime = constants::intake::kRotationPeakCurrentDuration;
        m_articulationConfig.CurrentLimits.SupplyCurrentLimitEnable = constants::intake::kRotationEnableCurrentLimit;

        m_articulationConfig.MotorOutput.Inverted = constants::intake::kArticulationMotorInverted;
        // m_articulationConfig.MotorOutput.NeutralMode = constants::intake::kRotationNeutralMode;

        m_articulationConfig.Slot0.kS = constants::intake::kArticulationKS;
        m_articulationConfig.Slot0.kV = constants::intake::kArticulationKV;
        m_articulationConfig.Slot0.kA = constants::intake::kArticulationKA;
        m_articulationConfig.Slot0.kP = constants::intake::kArticulationKP;
        m_articulationConfig.Slot0.kI = constants::intake::kArticulationKI;
        m_articulationConfig.Slot0.kD = constants::intake::kArticulationKD;
        // m_articulationConfig.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;

        m_articulationConfig.MotionMagic.MotionMagicCruiseVelocity = constants::intake::kMotionMagicMaxVelocity;
        m_articulationConfig.MotionMagic.MotionMagicAcceleration = constants::intake::kMotionMagicMaxAcceleration;
        m_articulationConfig.MotionMagic.MotionMagicJerk = constants::intake::kMotionMagicMaxJerk;

        m_intakeArticulationMotor.GetConfigurator().Apply(m_articulationConfig);
        m_intakeArticulationMotor.SetPosition(0.0_tr);
        m_intakeRollerMotor.GetConfigurator().Apply(m_rollerConfig);

        SetState(State::RetractIntake);
    }

    void Intake::SetIntakePosition(double position)
    {
        ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};
        frc::SmartDashboard::PutNumber("Intake target turns ", position / constants::intake::kRotationConversionRotationsToDegrees);
        m_intakeArticulationMotor.SetControl(request.WithEnableFOC(true).WithPosition(
            units::turn_t{position / constants::intake::kRotationConversionRotationsToDegrees}));
    }

    void Intake::SetDesiredRollerPower(double power) {
        m_desiredPower = power; 
    }

    void Intake::SetRollerPower(double power)
    {
        m_intakeRollerMotor.Set(power);
    }

    void Intake::SetRotationPower(double power) 
    {
        m_intakeArticulationMotor.Set(power); 
    }

    void Intake::SetState(State state)
    {
        m_state = state;
        // Figure out target angle then set rotation to this angle

        switch (state)
        {
        case (State::ExtendIntake):
        {
            m_targetAngle = constants::intake::kExtendAngle.value(); 
            // SetIntakePosition(constants::intake::kExtendAngle.value());
            // SetRollerPower(rollerPower);
            // SetRollerPower(rollerPower); // Set to extended position
            break;
        }
        case (State::MidIntake):
        {
            m_targetAngle = constants::intake::kMidAngle.value(); 
            // SetIntakePosition(constants::intake::kMidAngle.value());
            // SetRollerPower(rollerPower);
            // SetRollerPower(rollerPower); // Set to extended position
            break;
        }
        case (State::RetractIntake):
        {
            m_targetAngle = constants::intake::kRetractAngle.value(); 
            // SetIntakePosition(constants::intake::kRetractAngle.value());
            // SetRollerPower(0.0);
            // SetIntakePosition(constants::intake::kIntakeRetractedPosition); // Set to default position
            break;
        }
        default:
            break;
        }
        // Rotation will be set periodicially
    }

    State Intake::GetState()
    {
        frc::SmartDashboard::PutNumber("Intake State: ",  static_cast<int>(m_state));
        return m_state;
    }

    // This method will be called once per scheduler run
    void Intake::Periodic()
    {

        frc::SmartDashboard::PutNumber("Intake roller power (actual)", m_intakeRollerMotor.Get());
        frc::SmartDashboard::PutNumber("Intake roller power (desired)", m_intakeRollerMotor.GetClosedLoopReference(0).GetValue());
        frc::SmartDashboard::PutNumber("Intake roller power (m_desiredPower)", m_desiredPower); 

        frc::SmartDashboard::PutNumber("Intake Rotation Motor Turns ", m_intakeArticulationMotor.GetPosition().GetValue().value());
        frc::SmartDashboard::PutNumber("Intake Rotation Motor Degrees ", GetArticulationAngle().value());


        if (frc::Preferences::GetBoolean("Tuning Mode", false))
        {
        }

        frc::SmartDashboard::PutNumber("Intake target angle ", m_targetAngle);

        if (GetArticulationAngle() <= constants::intake::kAngleThresholdRollers) {
            m_desiredPower = 0.1; 
            // SetRollerPower(m_desiredPower); 
        } else {
            m_desiredPower = 0.0; 
            // SetRollerPower(0.0); 
        }
        SetIntakePosition(m_targetAngle); 
    }


    units::degree_t Intake::GetArticulationAngle() {
        double encoderValue = m_intakeArticulationMotor.GetPosition().GetValue().value(); 
        return units::degree_t{encoderValue * constants::intake::kRotationConversionRotationsToDegrees};
    }

    double Intake::GetTargetAngle() {
        return m_targetAngle; 
    }

}
