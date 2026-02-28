#pragma once 

#include <string>
#include <cmath> 

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>

#include "constants/swerve/SwerveDrive.h"
#include "constants/swerve/SwerveModule.h"
#include "constants/AutoAlign.h"
#include "constants/shooter/Articulation.h"



#include "util/Vector2d.h"

/*

TODO

TODO: REV UP MOTOR

Flow: TODO

*/


namespace util {
    namespace align {
        namespace hood {
            frc::Pose2d GetHubPose(); 

            bool BehindHub(frc::Pose2d robotPose);
            bool InNeutralZone(frc::Pose2d robotPose);
            bool InTrench(frc::Pose2d robotPose);
            units::degree_t CalculateArticulationAngle(frc::Pose2d robotPose);
        }; 
    };
};