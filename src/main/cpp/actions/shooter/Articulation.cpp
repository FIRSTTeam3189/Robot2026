#include "actions/shooter/Articulation.h"

namespace actions::shooter::articulation {

    frc2::CommandPtr SetArticulationToFixedState(subsystems::shooter::Articulation *articulation) {
        return commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::FixedAlign).ToPtr();
    }

    frc2::CommandPtr RunArticulationReverse(subsystems::shooter::Articulation *articulation) {
        return frc2::InstantCommand([articulation]() {
            articulation->SetArticulationMotorPower(-0.1); 
        }).ToPtr();
    }

    frc2::CommandPtr ResetArticulationPower(subsystems::shooter::Articulation *articulation) {
        return frc2::InstantCommand([articulation]() {
            articulation->SetArticulationMotorPower(0.0); 
        }).ToPtr();
    }

    frc2::CommandPtr Articulation20Degrees(subsystems::shooter::Articulation *articulation) {
        return commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::Degrees20).ToPtr(); 
    }

    frc2::CommandPtr Articulation25Degrees(subsystems::shooter::Articulation *articulation) {
        return commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::Degrees25).ToPtr(); 
    }

    frc2::CommandPtr Articulation30Degrees(subsystems::shooter::Articulation *articulation) {
        return commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::Degrees30).ToPtr(); 
    }

    frc2::CommandPtr Articulation40Degrees(subsystems::shooter::Articulation *articulation) {
        return commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::Degrees40).ToPtr(); 
    }


    frc2::CommandPtr SetTurretAndArticulationManual(subsystems::shooter::Articulation *articulation, subsystems::shooter::Turret *turret) {
        return frc2::SequentialCommandGroup(
            commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::FixedAlign),
            commands::util::SetTurretAngle(turret, constants::shooter::TurretTarget::PointStraight)
        ).ToPtr();
    }
}
