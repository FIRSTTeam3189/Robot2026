#pragma once


#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/util/SetFlywheelMotorPower.h"
#include "commands/util/SetFlywheelMotorVelocity.h"
#include "commands/util/SetLoaderPower.h"
#include "commands/util/SetShooterKickerPower.h"
#include "commands/util/SetIntakeState.h"
#include "commands/main/SetIntakeAngleAndPower.h"
#include "commands/util/SetHoodAngle.h"

#include "subsystems/intake/Intake.h"

#include "subsystems/shooter/Hood.h"
#include "subsystems/shooter/Turret.h"
#include "subsystems/indexer/Kicker.h"
#include "subsystems/shooter/Flywheel.h"
#include "subsystems/indexer/Loader.h"

#include "util/align/Drivetrain.h"

#include "constants/shooter/Turret.h"
#include "constants/shooter/Hood.h"
#include "constants/indexer/Loader.h"
#include "constants/indexer/Kicker.h"


namespace actions {
    namespace robot {
        frc2::SequentialCommandGroup Shoot(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel, 
                                           subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                           subsystems::shooter::Hood *articulation);

        frc2::CommandPtr ShootEnd(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker, 
                                  subsystems::indexer::Loader *loader, subsystems::shooter::Hood *articulation);
                                  
        frc2::CommandPtr ShootWithVelocity(subsystems::shooter::Flywheel *flywheel, double rpm);
        
        frc2::ParallelCommandGroup AdvancedShoot(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel, 
                                                 subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                                 subsystems::shooter::Hood *articulation);
            

        frc2::CommandPtr AdvancedShootEnd(subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                          subsystems::shooter::Hood *articulation);

       
        frc2::CommandPtr GoToClosestPose(subsystems::swerve::Drive *swerveDrive, pathplanner::PathConstraints constraints);

        frc2::CommandPtr NomNomModeOn(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel, 
                                      subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                      subsystems::intake::Intake *intake, subsystems::shooter::Hood *articulation);

        frc2::CommandPtr NomNomModeOff(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker, 
                                       subsystems::indexer::Loader *loader, subsystems::intake::Intake *intake);

        frc2::CommandPtr NomNomMode(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel, 
                                    subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                    subsystems::intake::Intake *intake, subsystems::shooter::Hood *articulation, bool *nomNomMode);

        frc2::CommandPtr ToggleShooterState(constants::ShooterState *shooterState);

        frc2::CommandPtr ShooterStateAutomatic(constants::ShooterState *shooterState);
        frc2::CommandPtr ShooterStateFixed(constants::ShooterState *shooterState);
        frc2::CommandPtr ShooterStateReset(constants::ShooterState *shooterState);
        frc2::CommandPtr ShooterStateManual(constants::ShooterState *shooterState);
        frc2::CommandPtr ShooterStateHoodAndInterpAuto(constants::ShooterState *shooterState);
    };
};