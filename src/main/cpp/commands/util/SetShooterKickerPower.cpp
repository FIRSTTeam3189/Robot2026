// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetShooterKickerPower.h"

namespace commands::util {
    SetShooterKickerPower::SetShooterKickerPower(subsystems::indexer::Kicker *kicker, double power)
      : m_kicker(kicker),
        m_power(power),
        m_timer()
    {
        AddRequirements(kicker);
    }
    
    void SetShooterKickerPower::Initialize() {
        // m_kicker->SetKickerMotorPower(m_power);
        m_timer.Start(); 
    }
    
    void SetShooterKickerPower::Execute() {
        units::time::second_t elapsed = m_timer.Get();
        frc::SmartDashboard::PutNumber("timer stuff/kicker timer seconds passed", elapsed.value()); 
        double targetPower = m_power;  
        if (elapsed >= 2.0_s && elapsed <= 2.5_s) { //unstuck balls for 0.5s
            targetPower = -m_power; 
        } else if (elapsed > 2.5_s) {
            m_timer.Restart(); 
        }
        m_kicker->SetKickerMotorPower(targetPower);
    }
    
    void SetShooterKickerPower::End(bool interrupted) {
        // m_kicker->SetKickerMotorPower(0.0);
    }
    
    bool SetShooterKickerPower::IsFinished() {
        // return true;
        return false; 
    }
}
