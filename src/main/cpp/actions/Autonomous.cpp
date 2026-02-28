#include "actions/Autonomous.h" 

namespace actions::autonomous {
    frc2::CommandPtr AutoShoot(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker, 
                               subsystems::indexer::Loader *loader, subsystems::shooter::Articulation *articulation) {
        return frc2::SequentialCommandGroup(
            // commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::Degrees20),
            frc2::ParallelCommandGroup(
            frc2::SequentialCommandGroup(
                commands::util::SetFlywheelMotorPower(flywheel, constants::shooter::flywheel::kAutoStaticFlywheelPower),
                frc2::WaitCommand(constants::shooter::flywheel::kFlywheelRunDelay)
            ),
            commands::util::SetLoaderPower(loader, -constants::indexer::loader::kLoadFuelPower)
            ),
            frc2::ParallelCommandGroup(
            commands::util::SetLoaderPower(loader, constants::indexer::loader::kLoadFuelPower),
            commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)
            )
        ).ToPtr();
    }

    frc2::CommandPtr AutoShootEnd(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker, 
                                  subsystems::indexer::Loader *loader, subsystems::shooter::Articulation *articulation) {
        return frc2::SequentialCommandGroup(
                commands::util::SetFlywheelMotorPower(flywheel, 0.0), 
                commands::util::SetLoaderPower(loader, 0.0),
                commands::util::SetShooterKickerPower(kicker, 0.0),
                commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::Degrees0)
            ).ToPtr();
    }
};
