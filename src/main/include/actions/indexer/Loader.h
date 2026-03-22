#pragma once 

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "commands/util/SetLoaderPower.h"

#include "subsystems/indexer/Loader.h"

#include "constants/indexer/Loader.h"


namespace actions {
    namespace indexer {
        namespace loader {  
            frc2::CommandPtr RunLoaderForward(subsystems::indexer::Loader *loader);
            frc2::CommandPtr RunLoaderBackward(subsystems::indexer::Loader *loader);
            frc2::CommandPtr ResetLoader(subsystems::indexer::Loader *loader);
            frc2::InstantCommand ResetLoaderInstant(subsystems::indexer::Loader *loader); 
        };
    };
};