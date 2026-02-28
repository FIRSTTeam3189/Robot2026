// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/util/SetFlywheelMotorVelocity.h"

namespace commands::util {
    SetFlywheelMotorVelocity::SetFlywheelMotorVelocity(subsystems::shooter::Flywheel *flywheel, units::turns_per_second_t velocity)
    : m_flywheel(flywheel),
        m_velocity(velocity)
    {
        AddRequirements(flywheel);
    }

    void SetFlywheelMotorVelocity::Execute() {
        m_flywheel->SetFlyWheelMotorVelocity(m_velocity);
    }

    void SetFlywheelMotorVelocity::End(bool interrupted) {
        // m_flywheel->SetFlywheelMotorPower(0.0);
    }

    bool SetFlywheelMotorVelocity::IsFinished() {
        return true;
    }
}
