#include "actions/shooter/Hood.h"

namespace actions::shooter::hood {

    frc2::CommandPtr SetArticulationToFixedState(subsystems::shooter::Hood *articulation) {
        return commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::FixedAlign).ToPtr();
    }

    frc2::CommandPtr RunArticulationForward(subsystems::shooter::Hood *articulation) {
        return frc2::InstantCommand([articulation]() {
            articulation->SetArticulationMotorPower(constants::shooter::hood::kArticulationMotorPowerForward); 
        }).ToPtr();
    }

    frc2::CommandPtr RunArticulationReverse(subsystems::shooter::Hood *articulation) {
        return frc2::InstantCommand([articulation]() {
            articulation->SetArticulationMotorPower(constants::shooter::hood::kArticulationMotorPowerReverse); 
        }).ToPtr();
    }

    frc2::CommandPtr ResetArticulationPower(subsystems::shooter::Hood *articulation) {
        return frc2::InstantCommand([articulation]() {
            articulation->SetArticulationMotorPower(0.0); 
        }).ToPtr();
    }

    frc2::CommandPtr Articulation0Degrees(subsystems::shooter::Hood *articulation) {
        return commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees0).ToPtr(); 
    }

    frc2::CommandPtr Articulation20Degrees(subsystems::shooter::Hood *articulation) {
        return commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees20).ToPtr(); 
    }

    frc2::CommandPtr Articulation25Degrees(subsystems::shooter::Hood *articulation) {
        return commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees25).ToPtr(); 
    }

    frc2::CommandPtr Articulation30Degrees(subsystems::shooter::Hood *articulation) {
        return commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees30).ToPtr(); 
    }

    frc2::CommandPtr Articulation40Degrees(subsystems::shooter::Hood *articulation) {
        return commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees40).ToPtr(); 
    }

    frc2::CommandPtr SetTurretAndArticulationManual(subsystems::shooter::Hood *articulation, subsystems::shooter::Turret *turret) {
        return frc2::SequentialCommandGroup(
            commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::FixedAlign),
            commands::util::SetTurretAngle(turret, constants::shooter::TurretTarget::PointStraight)
        ).ToPtr();
    }

    frc2::CommandPtr IncrementArticulationAngle(subsystems::shooter::Hood *articulation) {
        return frc2::InstantCommand([articulation]() {
            articulation->SetTargetAngle(articulation->GetTargetAngle() + 
                                               constants::shooter::hood::kArticulationAngleIncrement); 
        }, {articulation}).ToPtr();
    }

    frc2::CommandPtr DecrementArticulationAngle(subsystems::shooter::Hood *articulation) {
        return frc2::InstantCommand([articulation]() {
            articulation->SetTargetAngle(articulation->GetTargetAngle() + 
                                               constants::shooter::hood::kArticulationAngleDecrement); 
        }, {articulation}).ToPtr();
    }
}
