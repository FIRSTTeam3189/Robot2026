#pragma once 




#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/util/SetTurretAngle.h"
#include "commands/util/SetHoodAngle.h"

#include "subsystems/shooter/Hood.h"
#include "subsystems/shooter/Turret.h"
#include "constants/shooter/Turret.h"
#include "constants/shooter/Hood.h"


namespace actions {
    namespace shooter {
        namespace hood {
            frc2::CommandPtr SetArticulationToFixedState(subsystems::shooter::Hood * articulation); 
            frc2::CommandPtr RunArticulationForward(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr RunArticulationReverse(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr ResetArticulationPower(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr Articulation0Degrees(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr Articulation20Degrees(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr Articulation25Degrees(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr Articulation30Degrees(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr Articulation40Degrees(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr SetTurretAndArticulationManual(subsystems::shooter::Hood *articulation, subsystems::shooter::Turret *turret);
            frc2::CommandPtr IncrementArticulationAngle(subsystems::shooter::Hood *articulation);
            frc2::CommandPtr DecrementArticulationAngle(subsystems::shooter::Hood *articulation);

        };
    };
};