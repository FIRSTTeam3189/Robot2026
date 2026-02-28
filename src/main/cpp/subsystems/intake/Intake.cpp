#include "subsystems/intake/Intake.h"


namespace subsystems::intake {

    Intake::Intake() : 

    m_intakeRollerMotor(constants::intake::kRollerMotorID),
    m_intakeConfig(),
    m_intakeSolenoid(constants::intake::kSolenoidModule, constants::intake::kSolenoidType,
                    constants::intake::kSolenoidForwardChannel, constants::intake::kSolenoidReverseChannel),
    m_state(State::RetractIntake)


    {
        ConfigRollerMotor();

    }



    void Intake::ConfigRollerMotor() {
        m_intakeRollerMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

        m_intakeConfig.MotorOutput.Inverted = constants::intake::kRollerMotorInverted;
        m_intakeConfig.MotorOutput.NeutralMode = constants::intake::kRollerMotorNeutralMode;

        
        m_intakeRollerMotor.GetConfigurator().Apply(m_intakeConfig);
        // m_turretMotor.SetPosition(0.0_tr);
    }


    void Intake::SetRollerPower(double power){
        m_intakeRollerMotor.Set(power);
        m_desiredPower = power;
    }

    void Intake::SetIntake(double power) {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kForward);
        m_intakeRollerMotor.Set(power);
        m_desiredPower = power;
    }

    void Intake::RetractIntake() {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kReverse);
        m_intakeRollerMotor.Set(0.0);
        m_desiredPower = 0.0;
    }

    void Intake::SetState(State state, double rollerPower) {
        m_state = state;
        m_desiredPower = rollerPower;
        // Figure out target angle then set rotation to this angle


        switch (state) {
            case (State::ExtendIntake): {
                SetIntake(rollerPower);
                SetRollerPower(rollerPower);
                break;
            }
            case (State::RetractIntake): {
                RetractIntake();
                break;
            }   
            default:
                RetractIntake();
                break;  
        }
        // Rotation will be set periodicially
    }

    State Intake::GetState(){
        // frc::SmartDashboard::PutNumber("Intake State: ",  static_cast<int>(m_state));
        return m_state;
    }

    // This method will be called once per scheduler run
    void Intake::Periodic() {
        
        frc::SmartDashboard::PutNumber("Intake roller power (actual)", m_intakeRollerMotor.Get());
        frc::SmartDashboard::PutNumber("Intake roller power (desired)", m_desiredPower);
        // frc::SmartDashboard::PutNumber("Intake State", static_cast<int>(m_state));
        // frc::SmartDashboard::PutNumber("Algae Intake rotation getAppliedOutput", m_rotationMotor.GetAppliedOutput());

        if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
            
        }

    }

}

