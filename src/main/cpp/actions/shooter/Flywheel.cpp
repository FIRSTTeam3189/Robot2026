#include "actions/shooter/Flywheel.h"

namespace actions::shooter::flywheel {
    frc2::CommandPtr FlywheelPowerForward(subsystems::shooter::Flywheel *flywheel){
        return commands::util::SetFlywheelMotorPower(flywheel, -0.15).ToPtr();
    }

    frc2::CommandPtr FlywheelPowerReverse(subsystems::shooter::Flywheel *flywheel){
        return commands::util::SetFlywheelMotorPower(flywheel, 0.3).ToPtr();
    }

    frc2::CommandPtr ResetFlywheelPower(subsystems::shooter::Flywheel *flywheel){
        return commands::util::SetFlywheelMotorPower(flywheel, 0.0).ToPtr();
    }

    frc2::CommandPtr IncrementFlywheelVelocity(subsystems::shooter::Flywheel *flywheel) {
        return frc2::InstantCommand([flywheel]() {
            flywheel->SetFlywheelTargetRPS(flywheel->GetFlywheelTargetRPS() + constants::shooter::flywheel::kFlywheelIncrementVelocity.value());
        }, {flywheel}).ToPtr(); 
    }
    
    frc2::CommandPtr DecrementFlywheelVelocity(subsystems::shooter::Flywheel *flywheel) {
        return frc2::InstantCommand([flywheel]() {
            flywheel->SetFlywheelTargetRPS(flywheel->GetFlywheelTargetRPS() + constants::shooter::flywheel::kFlywheelDecrementVelocity.value());
        }, {flywheel}).ToPtr();
    }
};

