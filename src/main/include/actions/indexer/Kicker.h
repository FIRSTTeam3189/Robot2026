#pragma once 


#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "commands/util/SetShooterKickerPower.h"

#include "subsystems/indexer/Kicker.h"

#include "constants/indexer/Kicker.h"


namespace actions {
    namespace indexer {
        namespace kicker {  
            frc2::CommandPtr RunKickerForward(subsystems::indexer::Kicker *kicker);
            frc2::CommandPtr RunKickerBackward(subsystems::indexer::Kicker *kicker);
            frc2::CommandPtr ResetKicker(subsystems::indexer::Kicker *kicker);
            frc2::InstantCommand ResetKickerInstant(subsystems::indexer::Kicker *kicker);
        };
    };
};