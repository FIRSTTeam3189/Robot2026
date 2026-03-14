#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/util/SetFlywheelMotorPower.h"
#include "commands/util/SetLoaderPower.h"
#include "commands/util/SetShooterKickerPower.h"
#include "commands/util/SetHoodAngle.h"

#include "subsystems/shooter/Hood.h"
#include "subsystems/indexer/Kicker.h"
#include "subsystems/shooter/Flywheel.h"
#include "subsystems/indexer/Loader.h"

#include "constants/shooter/Flywheel.h"
#include "constants/shooter/Hood.h"
#include "constants/indexer/Loader.h"
#include "constants/indexer/Kicker.h"


namespace actions {
    namespace autonomous {
        frc2::CommandPtr AutoShoot(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker, 
                                    subsystems::indexer::Loader *loader, subsystems::shooter::Hood *articulation);
        
        frc2::CommandPtr AutoShootEnd(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker, 
                                        subsystems::indexer::Loader *loader, subsystems::shooter::Hood *articulation);
    };
};
