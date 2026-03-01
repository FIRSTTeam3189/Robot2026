#pragma once



#include <frc/DriverStation.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>


#include "subsystems/swerve/Drive.h"


#include "constants/swerve/SwerveDrive.h"
#include "constants/AutoAlign.h"
 

namespace actions {
    frc2::InstantCommand SetRobotPose(subsystems::swerve::Drive *swerveDrive, frc::Pose2d blueRobotPose, frc::Pose2d redRobotPose) ; 
    frc2::CommandPtr ResetPose(subsystems::swerve::Drive *swerveDrive);
    frc2::CommandPtr EmptyCommand(); 
    frc2::CommandPtr SetLeftTrenchAutoPose(subsystems::swerve::Drive *swerveDrive); 
    frc2::CommandPtr SetRightTrenchAutoPose(subsystems::swerve::Drive *swerveDrive);
}; 


