#pragma once 


#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>

#include "commands/util/SetIntakeRollerMotor.h"
#include "commands/util/SetIntakeManual.h"

#include "subsystems/intake/Intake.h"


#include "constants/Intake.h"

namespace actions {
    namespace intake {  
        commands::util::SetIntakeRollerMotor IntakePowerForward(subsystems::intake::Intake *intake);
        frc2::CommandPtr IntakePowerReverse(subsystems::intake::Intake *intake);
        frc2::CommandPtr ResetIntakePower(subsystems::intake::Intake *intake);

        frc2::CommandPtr IntakeForwardManual(subsystems::intake::Intake *intake); 
        frc2::CommandPtr IntakeBackwardManual(subsystems::intake::Intake *intake);
    };
};