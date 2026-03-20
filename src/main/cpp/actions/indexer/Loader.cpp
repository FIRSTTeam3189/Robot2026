#include "actions/indexer/Loader.h"

namespace actions::indexer::loader {
    frc2::CommandPtr RunLoaderForward(subsystems::indexer::Loader *loader) {
        return commands::util::SetLoaderPower(loader, constants::indexer::loader::kLoadFuelPower).ToPtr();
    }

    frc2::CommandPtr RunLoaderBackward(subsystems::indexer::Loader *loader) {
        return commands::util::SetLoaderPower(loader, constants::indexer::loader::kReverseRollerPower).ToPtr();
    }

    frc2::CommandPtr ResetLoader(subsystems::indexer::Loader *loader) {
        return commands::util::SetLoaderPower(loader, 0.0).ToPtr();
    }
}; 

