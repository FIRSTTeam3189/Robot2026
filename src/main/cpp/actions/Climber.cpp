#include "actions/Climber.h"

namespace actions::climber {
    frc2::CommandPtr PutClimberUp(subsystems::climber::Climber *climber) {
        return commands::util::SetClimberPower(climber, constants::climber::kClimberPowerUp).ToPtr();
    }

    frc2::CommandPtr StopClimber(subsystems::climber::Climber *climber) {
        return commands::util::SetClimberPower(climber, 0.0).ToPtr();
    }

    frc2::CommandPtr PutClimberDown(subsystems::climber::Climber *climber) {
        return commands::util::SetClimberPower(climber, constants::climber::kClimberPowerDown).ToPtr();
    }

    frc2::CommandPtr ResetClimber(subsystems::climber::Climber *climber) {
        return commands::util::SetClimberPower(climber, constants::climber::kResetClimberPower).ToPtr();
    }
}; 