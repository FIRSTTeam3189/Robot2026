#pragma once 




#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/util/SetTurretAngle.h"
#include "commands/util/SetArticulationAngle.h"

#include "subsystems/shooter/Articulation.h"
#include "subsystems/shooter/Turret.h"

#include "constants/shooter/Turret.h"
#include "constants/shooter/Articulation.h"


namespace actions {
    namespace shooter {
        namespace articulation {
            frc2::CommandPtr SetArticulationToFixedState(subsystems::shooter::Articulation * articulation); 
            // frc2::CommandPtr RunArticulationForward(subsystems::shooter::Articulation *articulation);
            frc2::CommandPtr RunArticulationReverse(subsystems::shooter::Articulation *articulation);
            frc2::CommandPtr ResetArticulationPower(subsystems::shooter::Articulation *articulation);
            frc2::CommandPtr Articulation20Degrees(subsystems::shooter::Articulation *articulation);
            frc2::CommandPtr Articulation25Degrees(subsystems::shooter::Articulation *articulation);
            frc2::CommandPtr Articulation30Degrees(subsystems::shooter::Articulation *articulation);
            frc2::CommandPtr Articulation40Degrees(subsystems::shooter::Articulation *articulation);
            frc2::CommandPtr SetTurretAndArticulationManual(subsystems::shooter::Articulation *articulation, subsystems::shooter::Turret *turret);
        };
    };
};