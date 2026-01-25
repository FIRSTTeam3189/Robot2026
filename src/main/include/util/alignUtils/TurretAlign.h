#pragma once 

#include <string>

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

#include "util/Vector2d.h"
#include "util/alignUtils/AutoAlign.h"


/*

TODO

TODO: REV UP MOTOR

Flow: TODO

*/
namespace TurretAlign {

    frc::Pose2d UpdateTargetWithVelocity(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d robotVelocity);

    frc::Pose2d CalculateTargetPose(frc::Pose2d robotPose);

    double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset); 

}; 