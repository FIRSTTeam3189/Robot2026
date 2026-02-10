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
#include "commands/utilCommands/SetLoaderPower.h"
#include "commands/utilCommands/SetShooterKickerPower.h"
#include "commands/utilCommands/SetShooterRotationPower.h"
#include "commands/utilCommands/SetTurretAngle.h"
#include "commands/utilCommands/SetIntakeRollerMotor.h"
#include "commands/utilCommands/SetArticulationAngle.h"
#include "commands/mainCommands/SetIntakeAngleAndPower.h"
#include "commands/utilCommands/SetIntakeManual.h"

#include "util/alignUtils/AutoAlign.h"
#include "util/LinearInterpolator.h"
#include "util/interpolatorDataUtils/InterpolatorData.h"

#include "subsystems/PoseEstimatorHelper.h"
#include "subsystems/PhotonVision.h"
#include "subsystems/Intake.h"
#include "subsystems/shooterSubsystem/Articulation.h"
#include "subsystems/shooterSubsystem/Turret.h"
#include "subsystems/shooterSubsystem/Kicker.h"
#include "subsystems/shooterSubsystem/Flywheel.h"
#include "subsystems/shooterSubsystem/Loader.h"




// #include "subsystems/Limelight.h"

#include "Constants/OperatorConstants.h"
#include "Constants/AutoConstants.h"
#include "Constants/SwerveDriveConstants.h"
#include "Constants/shooterConstants/TurretConstants.h"
#include "Constants/shooterConstants/ArticulationConstants.h"
#include "Constants/shooterConstants/LoaderConstants.h"
#include "Constants/shooterConstants/KickerConstants.h"
#include "Constants/InterpolatorConstants.h"
#include "Constants/VisionConstants.h"



#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>



namespace CommandManager {

    frc2::CommandPtr ResetPose(SwerveDrive *swerveDrive);

    frc2::CommandPtr TurretPowerForward(Turret *turret);
    frc2::CommandPtr TurretPowerReverse(Turret *turret);
    frc2::CommandPtr ResetTurretPower(Turret *turret);

    frc2::CommandPtr AlignToHub(Turret *turret, Articulation *articulation);

    frc2::CommandPtr PointTurretStraight(Turret *turret);
    frc2::CommandPtr PointTurretLeft(Turret *turret);
    frc2::CommandPtr PointTurretRight(Turret *turret);

    frc2::CommandPtr SlowDownSwerves(SwerveDrive *swerveDrive, DriveState *driveState, frc::PS5Controller *bill);
    frc2::CommandPtr DefaultSwerveSpeed(SwerveDrive *swerveDrive, DriveState *driveState, frc::PS5Controller *bill);
    frc2::CommandPtr ClimbAlignSwerves(SwerveDrive *swerveDrive, DriveState *driveState, frc::PS5Controller *bill);

    frc2::CommandPtr IntakePowerForward(Intake *Intake);
    frc2::CommandPtr IntakePowerReverse(Intake *Intake);
    frc2::CommandPtr ResetIntakePower(Intake *Intake);

    frc2::CommandPtr FlywheelPowerForward(Flywheel *flywheel);
    frc2::CommandPtr FlywheelPowerReverse(Flywheel *flywheel);
    frc2::CommandPtr ResetFlywheelPower(Flywheel *flywheel);
    frc2::CommandPtr IncrementFlywheelPower(Flywheel *flywheel); 
    frc2::CommandPtr DecrementFlywheelPower(Flywheel *flywheel);

    frc2::CommandPtr ArticulationAngleUp(Articulation *articulation);
    frc2::CommandPtr ArticulationAngleDown(Articulation *articulation);
    frc2::CommandPtr SetArticulationToFixedState(Articulation* articulation); 

    frc2::CommandPtr SetTurretAndArticulationManual(Articulation *articulation, Turret *turret); 

    frc2::CommandPtr RunKickerForward(Kicker *Kicker);
    frc2::CommandPtr RunKickerBackward(Kicker *Kicker);
    frc2::CommandPtr ResetKicker(Kicker *Kicker);

    frc2::CommandPtr RunLoaderForward(Loader *loader);
    frc2::CommandPtr RunLoaderBackward(Loader *loader);
    frc2::CommandPtr ResetLoader(Loader *loader);


    frc2::CommandPtr Shoot(Flywheel *flywheel, Kicker *kicker, Loader *loader, Articulation *articulation);
    frc2::CommandPtr ShootEnd(Flywheel *flywheel, Kicker *kicker, Loader *loader, Articulation *articulation);
      
    frc2::CommandPtr AddInterpolatorPoint(InterpolatorData *interpolatorData, SwerveDrive *swerveDrive, 
                                          Flywheel *flywheel, Articulation *articulation); 
    // frc2::CommandPtr RunFlywheelAndLoaderTest(Kicker *Kicker, Flywheel *flywheel);
    frc2::CommandPtr SaveAllInterpolationPoints(InterpolatorData *interpolatorData);
    frc2::CommandPtr RemoveLastInterpolationPoint(InterpolatorData *interpolatorData);

    frc2::CommandPtr GoToClosestPose(SwerveDrive *swerveDrive, pathplanner::PathConstraints constraints);

    frc2::CommandPtr NomNomModeOn(Flywheel *flywheel, Kicker *kicker, Loader *loader, Intake *intake);
    frc2::CommandPtr NomNomModeOff(Flywheel *flywheel, Kicker *kicker, Loader *loader, Intake *intake);

    frc2::CommandPtr EmptyCommand(); 
};