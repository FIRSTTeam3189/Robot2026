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

    frc2::CommandPtr IntakeExtendState(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeState(intake, subsystems::intake::State::ExtendIntake, constants::intake::kRollerIntakePower).ToPtr();
    }

    frc2::CommandPtr IntakeRetractState(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeState(intake, subsystems::intake::State::RetractIntake, 0.0).ToPtr();
    }
}; 

