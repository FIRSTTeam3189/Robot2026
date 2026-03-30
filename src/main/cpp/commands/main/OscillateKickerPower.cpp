// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/main/OscillateKickerPower.h"

namespace commands::main {
    OscillateKickerPower::OscillateKickerPower(subsystems::indexer::Kicker *kicker, double power)
      : m_kicker(kicker),
        m_power(power),
        m_timer()
    {
        AddRequirements(kicker);
    }
    
    void OscillateKickerPower::Initialize() {
        m_timer.Start(); 
    }
    
    void OscillateKickerPower::Execute() {
        units::time::second_t elapsed = m_timer.Get();
        frc::SmartDashboard::PutNumber("timer stuff/kicker timer seconds passed", elapsed.value()); 
        
        units::time::second_t periodStart = constants::indexer::kicker::kOscillatePeriod; 
        units::time::second_t periodStop = periodStart + constants::indexer::kicker::kOscillateDuration; 
        
        double targetPower = m_power;  
        if (elapsed >= periodStart && elapsed <= periodStop) { //unstuck balls for 0.5s
            targetPower = -0.95; 
        } else if (elapsed > periodStop) {
            m_timer.Restart(); 
        }

        m_kicker->SetKickerMotorPower(targetPower);
    }
    
    void OscillateKickerPower::End(bool interrupted) {
    }
    
    bool OscillateKickerPower::IsFinished() {
        return false; 
    }
}
