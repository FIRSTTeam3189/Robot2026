#include "subsystems/Intake.h"

Intake::Intake() : 

 m_rollerMotor(IntakeConstants::kRollerMotorID, rev::spark::SparkMax::MotorType::kBrushless),
 m_rollerConfig()

{
    ConfigRollerMotor();

}


void Intake::ConfigRollerMotor() {
    m_rollerConfig
        .Inverted(IntakeConstants::kRollerMotorInverted)
        .SmartCurrentLimit(IntakeConstants::kRollerMotorCurrentLimit)
        .SetIdleMode(IntakeConstants::kRollerMotorIdleMode);
    m_rollerMotor.Configure(m_rollerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void Intake::SetRollerPower(double power){
    m_rollerMotor.Set(power);
}



// This method will be called once per scheduler run
void Intake::Periodic() {
    
    frc::SmartDashboard::PutNumber("Algae Intake roller power", m_rollerMotor.Get());
    // frc::SmartDashboard::PutNumber("Algae Intake rotation getAppliedOutput", m_rotationMotor.GetAppliedOutput());

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        
    }

}

