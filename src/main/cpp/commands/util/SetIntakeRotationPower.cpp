// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetIntakeRotationPower.h"

namespace commands::util {
    SetIntakeRotationPower::SetIntakeRotationPower(subsystems::intake::Intake *intake, double power)
      : m_intake(intake),
        m_power(power)
    {
        AddRequirements(intake);
    }
    
    void SetIntakeRotationPower::Execute() {
        m_intake->SetRotationPower(m_power);
    }
    
    void SetIntakeRotationPower::End(bool interrupted) {
        m_intake->SetRotationPower(0.0);
    }
    
    bool SetIntakeRotationPower::IsFinished() {
        return false;
    }
}
