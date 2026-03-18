#include "actions/Intake.h"

namespace actions::intake {

    commands::util::SetIntakeRollerMotor IntakePowerForward(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeRollerMotor(intake, constants::intake::kRollerIntakePower);
    }

    frc2::CommandPtr IntakePowerReverse(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeRollerMotor(intake, constants::intake::kRollerReverseIntakePower).ToPtr();
    }

    frc2::CommandPtr ResetIntakePower(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeRollerMotor(intake, 0.0).ToPtr();
    }

    frc2::CommandPtr IntakeRotationPowerForward(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeRotationPower(intake, constants::intake::kRotationIntakePowerForward).ToPtr();
    }

    frc2::CommandPtr IntakeRotationPowerReverse(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeRotationPower(intake, constants::intake::kRotationIntakePowerReverse).ToPtr();
    }
    
    frc2::CommandPtr ResetIntakeRotationPower(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeRotationPower(intake, 0.0).ToPtr();
    }

    frc2::CommandPtr IntakeExtendState(subsystems::intake::Intake *intake) {
        return frc2::InstantCommand([intake]() {
                intake->SetState(subsystems::intake::State::ExtendIntake);
            }, {intake}).ToPtr(); 
    }

    frc2::CommandPtr IntakeMidState(subsystems::intake::Intake *intake) {
        return frc2::InstantCommand([intake]() {
            intake->SetState(subsystems::intake::State::MidIntake);
            // intake->SetRollerPower(constants::intake::kRollerIntakePower); 
        }, {intake}).ToPtr(); 
    }

    frc2::CommandPtr IntakeRetractState(subsystems::intake::Intake *intake) {
        return frc2::InstantCommand([intake]() {
            intake->SetState(subsystems::intake::State::RetractIntake);
            // intake->SetRollerPower(0.0); 
        }, {intake}).ToPtr();     
    }

    frc2::CommandPtr ToggleIntakeState(subsystems::intake::Intake *intake) {
        return commands::util::SetIntakeState(intake, constants::intake::kRollerIntakePower).ToPtr();
    }
    
}; 

