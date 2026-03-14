#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "commands/util/SetShooterRotationPower.h"
#include "commands/util/SetTurretAngle.h"
#include "commands/util/SetHoodAngle.h"

#include "subsystems/shooter/Hood.h"
#include "subsystems/shooter/Turret.h"
#include "constants/shooter/Turret.h"
#include "constants/shooter/Hood.h"


namespace actions {
    namespace shooter {
        namespace turret {  
            frc2::CommandPtr TurretPowerForward(subsystems::shooter::Turret *turret);
            frc2::CommandPtr TurretPowerReverse(subsystems::shooter::Turret *turret);
            frc2::CommandPtr ResetTurretPower(subsystems::shooter::Turret *turret);
            frc2::CommandPtr AlignToHub(subsystems::shooter::Turret *turret, subsystems::shooter::Hood *articulation);
            frc2::CommandPtr PointTurretStraight(subsystems::shooter::Turret *turret);
            frc2::CommandPtr PointTurretLeft(subsystems::shooter::Turret *turret);
            frc2::CommandPtr PointTurretRight(subsystems::shooter::Turret *turret);
        };
    };
};
