#include "actions/indexer/Kicker.h"

namespace actions::indexer::kicker {
    frc2::CommandPtr RunKickerForward(subsystems::indexer::Kicker *kicker) {
        return commands::util::SetShooterKickerPower(kicker, 0.4).ToPtr();
    }

    frc2::CommandPtr RunKickerBackward(subsystems::indexer::Kicker *kicker) {
        return commands::util::SetShooterKickerPower(kicker, -0.4).ToPtr();
    }

    frc2::CommandPtr ResetKicker(subsystems::indexer::Kicker *kicker) {
        return commands::util::SetShooterKickerPower(kicker, 0.0).ToPtr();
    }
};

