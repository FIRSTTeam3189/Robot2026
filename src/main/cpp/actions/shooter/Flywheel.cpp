#include "actions/shooter/Flywheel.h"

namespace actions::shooter::flywheel {
    frc2::CommandPtr FlywheelPowerForward(subsystems::shooter::Flywheel *flywheel){
        return commands::util::SetFlywheelMotorPower(flywheel, -0.85).ToPtr();
    }

    frc2::CommandPtr FlywheelPowerReverse(subsystems::shooter::Flywheel *flywheel){
        return commands::util::SetFlywheelMotorPower(flywheel, 0.90).ToPtr();
    }

    frc2::CommandPtr ResetFlywheelPower(subsystems::shooter::Flywheel *flywheel){
        return commands::util::SetFlywheelMotorPower(flywheel, 0.0).ToPtr();
    }

    frc2::CommandPtr IncrementFlywheelVelocity(subsystems::shooter::Flywheel *flywheel) {
        return frc2::InstantCommand([flywheel]() {
            flywheel->SetFlywheelTargetRPM(flywheel->GetFlywheelTargetRPM() + constants::shooter::flywheel::kFlywheelIncrementVelocity);
            // flywheel->SetFlywheelTargetRPM(300);
        }, {flywheel}).ToPtr(); 
    }
    
    frc2::CommandPtr DecrementFlywheelVelocity(subsystems::shooter::Flywheel *flywheel) {
        return frc2::InstantCommand([flywheel]() {
            flywheel->SetFlywheelTargetRPM(flywheel->GetFlywheelTargetRPM() + constants::shooter::flywheel::kFlywheelDecrementVelocity);
        }, {flywheel}).ToPtr();
    }

    frc2::CommandPtr TestFlywheelFixedVelocity(subsystems::shooter::Flywheel *flywheel) {
        return commands::util::SetFlywheelMotorVelocity(flywheel, 5000).ToPtr(); 
    }

};


