#pragma once



#include <frc/DriverStation.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/swerve/Drive.h"


#include "constants/swerve/SwerveDrive.h"
#include "constants/AutoAlign.h"
 

namespace actions {
    frc2::CommandPtr ResetPose(subsystems::swerve::Drive *swerveDrive);
    frc2::CommandPtr EmptyCommand(); 
}; 


