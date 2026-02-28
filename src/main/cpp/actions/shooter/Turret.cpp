#include "actions/shooter/Turret.h"

namespace actions::shooter::turret {
    frc2::CommandPtr TurretPowerForward(subsystems::shooter::Turret *turret) {
        return commands::util::SetShooterRotationPower(turret, 0.175).ToPtr(); 
    }

    frc2::CommandPtr TurretPowerReverse(subsystems::shooter::Turret *turret) {
        return commands::util::SetShooterRotationPower(turret, -0.175).ToPtr(); 
    }

    frc2::CommandPtr ResetTurretPower(subsystems::shooter::Turret *turret) {
        return commands::util::SetShooterRotationPower(turret, 0.0).ToPtr(); 
    }

    frc2::CommandPtr AlignToHub(subsystems::shooter::Turret *turret, subsystems::shooter::Articulation *articulation) {
        return frc2::ParallelCommandGroup(
            commands::util::SetTurretAngle(turret, constants::shooter::TurretTarget::AlignHub),
            commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::AligntoHub)
        ).ToPtr(); 
    }

    frc2::CommandPtr PointTurretStraight(subsystems::shooter::Turret *turret) {
        return commands::util::SetTurretAngle(turret, constants::shooter::TurretTarget::PointStraight).ToPtr(); 
    }

    frc2::CommandPtr PointTurretLeft(subsystems::shooter::Turret *turret) {
        return commands::util::SetTurretAngle(turret, constants::shooter::TurretTarget::Point45Deg).ToPtr();
    }

    frc2::CommandPtr PointTurretRight(subsystems::shooter::Turret *turret) {
        return commands::util::SetTurretAngle(turret, constants::shooter::TurretTarget::Point135Deg).ToPtr();
    }
};

