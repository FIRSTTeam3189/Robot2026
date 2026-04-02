#pragma once 

#include <string>

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>

#include "constants/swerve/SwerveDrive.h"
#include "constants/swerve/SwerveModule.h"
#include "constants/shooter/Turret.h"
#include "constants/AutoAlign.h"

#include "util/Vector2d.h"

/*

TODO

TODO: REV UP MOTOR

Flow: TODO

*/


namespace util {
    namespace align {
        namespace turret {
            frc::Pose2d UpdateTargetWithVelocity(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d robotVelocity, 
                                                units::angular_velocity::radians_per_second_t angularVelocity, frc::Field2d &velocityField);
            frc::Pose2d CalculateTargetPose(frc::Pose2d robotPose);
            units::meter_t GetDistanceToTarget(frc::Pose2d robotPose);
            double GetTargetAngle(frc::Pose2d robotPose, frc::Pose2d targetPose, Vector2d turretOffset, 
                                frc::Field2d &turretField, frc::Field2d &targetField); 
        }; 
    };
};