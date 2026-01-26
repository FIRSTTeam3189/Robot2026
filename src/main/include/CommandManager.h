#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>
#include <frc/PS5Controller.h>
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "commands/mainCommands/Drive.h"
#include "commands/utilCommands/SetFlywheelMotorPower.h"
#include "commands/utilCommands/SetShooterLoaderPower.h"
#include "commands/utilCommands/SetShooterRotationPower.h"
#include "commands/utilCommands/SetTurretAngle.h"
#include "commands/utilCommands/SetArticulationAngle.h"

#include "util/alignUtils/AutoAlign.h"
#include "util/PowerInterpolator.h"


#include "subsystems/PoseEstimatorHelper.h"
#include "subsystems/shooterSubsystem/Turret.h"
#include "subsystems/PhotonVision.h"
#include "subsystems/shooterSubsystem/Articulation.h"
#include "subsystems/shooterSubsystem/Loader.h"
#include "subsystems/shooterSubsystem/Flywheel.h"



// #include "subsystems/Limelight.h"

#include "Constants/OperatorConstants.h"
#include "Constants/AutoConstants.h"
#include "Constants/SwerveDriveConstants.h"
#include "Constants/shooterConstants/TurretConstants.h"
#include "Constants/shooterConstants/ArticulationConstants.h"
#include "Constants/VisionConstants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>


namespace CommandManager {
    frc2::CommandPtr ResetPose(SwerveDrive *swerveDrive);
};