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

#include "Constants/SwerveDriveConstants.h"
#include "Constants/SwerveModuleConstants.h"
#include "Constants/AutoAlignConstants.h"
#include "Constants/HoodConstants.h"

#include "util/Vector2d.h"
#include "util/alignUtils/AutoAlign.h"

/*

TODO

TODO: REV UP MOTOR

Flow: TODO

*/
namespace HoodAlign {

    frc::Pose2d GetHubPose(); 

    double GetDistanceToHub(frc::Pose2d robotPose);

}; 