#pragma once 

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "commands/util/SetFlywheelMotorPower.h"
#include "commands/util/SetFlywheelMotorVelocity.h"

#include "subsystems/shooter/Flywheel.h"

#include "constants/shooter/Flywheel.h"


namespace actions {
    namespace shooter {
        namespace flywheel {  
            frc2::CommandPtr FlywheelPowerForward(subsystems::shooter::Flywheel *flywheel);
            frc2::CommandPtr FlywheelPowerReverse(subsystems::shooter::Flywheel *flywheel);
            frc2::CommandPtr ResetFlywheelPower(subsystems::shooter::Flywheel *flywheel);
            frc2::CommandPtr IncrementFlywheelVelocity(subsystems::shooter::Flywheel *flywheel); 
            frc2::CommandPtr DecrementFlywheelVelocity(subsystems::shooter::Flywheel *flywheel); 
        };
    };
};