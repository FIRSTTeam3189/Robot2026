// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetShooterKickerPower.h"

namespace commands::util {
    SetShooterKickerPower::SetShooterKickerPower(subsystems::indexer::Kicker *kicker, double power)
      : m_kicker(kicker),
        m_power(power)
    {
        AddRequirements(kicker);
    }
    
    void SetShooterKickerPower::Initialize() {
        
    }
    
    void SetShooterKickerPower::Execute() {
        m_kicker->SetKickerMotorPower(m_power);
    }
    
    void SetShooterKickerPower::End(bool interrupted) {
        // m_kicker->SetKickerMotorPower(0.0);
    }
    
    bool SetShooterKickerPower::IsFinished() {
        return true;
    }
}
