#include "actions/Intake.h"

namespace actions::intake {

    commands::util::SetIntakeRollerMotor IntakePowerForward(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeRollerMotor(intake, constants::intake::kRollerIntakePower);
    }

    commands::util::SetIntakeRollerMotor IntakePowerReverse(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeRollerMotor(intake, constants::intake::kRollerReverseIntakePower);
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

    frc2::InstantCommand IntakeExtendState(subsystems::intake::Intake *intake) {
        return frc2::InstantCommand([intake]() {
            intake->SetDesiredRollerPower(constants::intake::kRollerIntakePower); 
            intake->SetState(subsystems::intake::State::ExtendIntake);
        }, {intake}); 
    }
    
    
    frc2::InstantCommand IntakeMidState(subsystems::intake::Intake *intake) {
        return frc2::InstantCommand([intake]() {
            intake->SetState(subsystems::intake::State::MidIntake);
            // intake->SetRollerPower(constants::intake::kRollerIntakePower); 
        }, {intake}); 
    }
    
    frc2::InstantCommand IntakeRetractState(subsystems::intake::Intake *intake) {
        return frc2::InstantCommand([intake]() {
            intake->SetDesiredRollerPower(0.0); 
            intake->SetState(subsystems::intake::State::RetractIntake);
        }, {intake});     
    }
    
    frc2::InstantCommand IntakeShakeUpState(subsystems::intake::Intake *intake) {
        return frc2::InstantCommand([intake]() {
            intake->SetDesiredRollerPower(0.0); 
            intake->SetState(subsystems::intake::State::ShakeIntakeUp);
        }, {intake}); 
    }

    frc2::InstantCommand IntakeShakeDownState(subsystems::intake::Intake *intake) {
        return frc2::InstantCommand([intake]() {
            intake->SetDesiredRollerPower(constants::intake::kRollerIntakePower); 
            intake->SetState(subsystems::intake::State::ShakeIntakeDown);
        }, {intake}); 
    }

    frc2::RepeatCommand ShakeIntake(subsystems::intake::Intake *intake) {
        return frc2::RepeatCommand(
            frc2::SequentialCommandGroup(
                actions::intake::IntakeShakeUpState(intake),
                frc2::WaitCommand(constants::intake::kShakeDelay),
                actions::intake::IntakeShakeDownState(intake),
                frc2::WaitCommand(constants::intake::kShakeDelay)
            )
        ); 
    }
    
    frc2::CommandPtr ToggleIntakeState(subsystems::intake::Intake *intake) {
        return commands::util::SetIntakeState(intake, constants::intake::kRollerIntakePower).ToPtr();
    }
    
}; 

