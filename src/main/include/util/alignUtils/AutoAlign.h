#pragma once 

#include "subsystems/SwerveDrive.h"

#include <string>

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>

#include "Constants/SwerveDriveConstants.h"
#include "Constants/AutoAlignConstants.h"

#include "util/Vector2d.h"



/*

This auto align chooses the closest one of n points (each with a pose2d) and runs the 
Pathfinding to pose to go there

TODO: REV UP MOTOR

Flow: Hold trigger -> aligns + ramps up (the shooter motor command internally should check RPM and run loader command when ready)

*/

namespace AutoAlign {

    frc::Pose2d GetClosestRobotPose(frc::Pose2d robotPose);

    frc2::CommandPtr AutoAlignToPose(SwerveDrive* swerveDrive, pathplanner::PathConstraints constraints);

    int GetIndexOfPathfindingPose(frc::Pose2d pathfindingPose);

}; 

