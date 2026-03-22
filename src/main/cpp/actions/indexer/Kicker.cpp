#include "actions/indexer/Kicker.h"

namespace actions::indexer::kicker {
    frc2::CommandPtr RunKickerForward(subsystems::indexer::Kicker *kicker) {
        return commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower).ToPtr();
    }

    frc2::CommandPtr RunKickerBackward(subsystems::indexer::Kicker *kicker) {
        return commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPowerReverse).ToPtr();
    }

    frc2::CommandPtr ResetKicker(subsystems::indexer::Kicker *kicker) {
        return commands::util::SetShooterKickerPower(kicker, 0.0).ToPtr();
    }

    frc2::InstantCommand ResetKickerInstant(subsystems::indexer::Kicker *kicker) {
        return frc2::InstantCommand([kicker]() {
            kicker->SetKickerMotorPower(0.0); 
        }, {kicker}); 
    }
};

