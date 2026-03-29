#pragma once 


#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RepeatCommand.h> 
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>

#include "commands/util/SetIntakeRollerMotor.h"
#include "commands/util/SetIntakeState.h"
#include "commands/util/SetIntakeRotationPower.h"

#include "subsystems/intake/Intake.h"


#include "constants/Intake.h"

namespace actions {
    namespace intake {  
        commands::util::SetIntakeRollerMotor IntakePowerForward(subsystems::intake::Intake *intake);
        commands::util::SetIntakeRollerMotor IntakePowerReverse(subsystems::intake::Intake *intake);
        frc2::CommandPtr ResetIntakePower(subsystems::intake::Intake *intake);

        frc2::CommandPtr IntakeRotationPowerForward(subsystems::intake::Intake *intake);
        frc2::CommandPtr IntakeRotationPowerReverse(subsystems::intake::Intake *intake);
        frc2::CommandPtr ResetIntakeRotationPower(subsystems::intake::Intake *intake);

        frc2::InstantCommand IntakeExtendState(subsystems::intake::Intake *intake); 
        frc2::InstantCommand IntakeMidState(subsystems::intake::Intake *intake);
        frc2::InstantCommand IntakeRetractState(subsystems::intake::Intake *intake);
        frc2::InstantCommand IntakeShakeUpState(subsystems::intake::Intake *intake);
        frc2::InstantCommand IntakeShakeDownState(subsystems::intake::Intake *intake);

        frc2::RepeatCommand ShakeIntake(subsystems::intake::Intake *intake);

        frc2::CommandPtr ToggleIntakeState(subsystems::intake::Intake *intake); 

    };
};