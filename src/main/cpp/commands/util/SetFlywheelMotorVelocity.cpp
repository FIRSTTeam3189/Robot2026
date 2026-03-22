// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetFlywheelMotorVelocity.h"

namespace commands::util {
    SetFlywheelMotorVelocity::SetFlywheelMotorVelocity(subsystems::shooter::Flywheel *flywheel, double velocity)
    : m_flywheel(flywheel),
        m_velocity(velocity)
    {
        SetName("SetFlywheelMotorVelocity"); 
        AddRequirements(flywheel);
    }

    void SetFlywheelMotorVelocity::Initialize() {
    }
    
    void SetFlywheelMotorVelocity::Execute() {
        m_flywheel->SetFlywheelMotorVelocityToTarget();
    }
    
    void SetFlywheelMotorVelocity::End(bool interrupted) {
        // m_flywheel->SetFlywheelMotorVelocity(0.0);
    }

    bool SetFlywheelMotorVelocity::IsFinished() {
        return false;
    }
}
