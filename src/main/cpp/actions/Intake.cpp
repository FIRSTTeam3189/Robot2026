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

    frc2::CommandPtr IntakeForwardManual(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeManual(intake, subsystems::intake::State::ExtendIntake, constants::intake::kRollerIntakePower).ToPtr();
    }

    frc2::CommandPtr IntakeBackwardManual(subsystems::intake::Intake *intake){
        return commands::util::SetIntakeManual(intake, subsystems::intake::State::RetractIntake, 0.0).ToPtr();
    }
}; 

