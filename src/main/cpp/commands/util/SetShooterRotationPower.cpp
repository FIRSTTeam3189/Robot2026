// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetShooterRotationPower.h"

namespace commands::util {
    SetShooterRotationPower::SetShooterRotationPower(subsystems::shooter::Turret *shooter, double power)
      : m_turret(shooter),
        m_power(power)
    {
        AddRequirements(shooter);
    }
    
    void SetShooterRotationPower::Execute() {
        m_turret->SetTurretRotationMotorPower(m_power);
    }
    
    void SetShooterRotationPower::End(bool interrupted) {
        m_turret->SetTurretRotationMotorPower(0.0);
    }
    
    bool SetShooterRotationPower::IsFinished() {
        return false;
    }
}
