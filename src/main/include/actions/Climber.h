#pragma once 

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>

#include "commands/util/SetClimberPower.h"

#include "subsystems/climber/Climber.h"

#include "constants/Climber.h"


namespace actions {
    namespace climber {
        frc2::CommandPtr PutClimberUp(subsystems::climber::Climber *climber);
        frc2::CommandPtr PutClimberDown(subsystems::climber::Climber *climber);
        frc2::CommandPtr StopClimber(subsystems::climber::Climber *climber);
        frc2::CommandPtr ResetClimber(subsystems::climber::Climber *climber);
    };
};