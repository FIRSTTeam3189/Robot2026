#include "actions/Robot.h"

namespace actions::robot {
    frc2::SequentialCommandGroup Shoot(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel, 
        subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, subsystems::shooter::Articulation *articulation) {
        return frc2::SequentialCommandGroup(
        // SetTurretAngle(turret, TurretTarget::AlignHub),
        // SetArticulationAngle(articulation, ArticulationTarget::Degrees20),
            frc2::ParallelCommandGroup(
                frc2::SequentialCommandGroup(
                    commands::util::SetFlywheelMotorVelocity(flywheel, constants::shooter::flywheel::kFlywheelFixedVelocity),
                    frc2::WaitCommand(constants::shooter::flywheel::kFlywheelRunDelay)
                ),
                commands::util::SetLoaderPower(loader, -constants::indexer::loader::kLoadFuelPower)
            ),
            frc2::ParallelCommandGroup(
                commands::util::SetLoaderPower(loader, constants::indexer::loader::kLoadFuelPower),
                commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)
            )
        );
    }


    frc2::CommandPtr ShootEnd(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker, 
        subsystems::indexer::Loader *loader, subsystems::shooter::Articulation *articulation) {
        return frc2::SequentialCommandGroup(
            // commands::util::SetFlywheelMotorPower(flywheel, 0.0), 
            commands::util::SetLoaderPower(loader, 0.0),
            commands::util::SetShooterKickerPower(kicker, 0.0),
            commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::Degrees0)
        ).ToPtr();
    }

    frc2::CommandPtr ShootWithVelocity(subsystems::shooter::Flywheel *flywheel, units::turns_per_second_t tps) {
        return commands::util::SetFlywheelMotorVelocity(flywheel, tps).ToPtr();
    }
        
    frc2::ParallelCommandGroup AdvancedShoot(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel, 
        subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, subsystems::shooter::Articulation *articulation) {
        return frc2::ParallelCommandGroup(
            frc2::SequentialCommandGroup(
                frc2::WaitCommand(constants::indexer::loader::kShootDelay),
                commands::util::SetLoaderPower(loader, constants::indexer::loader::kLoadFuelPower),
                commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)
            ),
            commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::AligntoHub)
        );
    }
         


    frc2::CommandPtr AdvancedShootEnd(subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                               subsystems::shooter::Articulation *articulation) {
        return frc2::SequentialCommandGroup(
            commands::util::SetLoaderPower(loader, 0.0),
            commands::util::SetShooterKickerPower(kicker, 0.0),
            commands::util::SetArticulationAngle(articulation, constants::shooter::ArticulationTarget::Degrees0)
        ).ToPtr();
    }   
    
    frc2::CommandPtr GoToClosestPose(subsystems::swerve::Drive *swerveDrive, pathplanner::PathConstraints constraints) {
        return frc2::cmd::RunOnce([swerveDrive, constraints](){
            frc2::CommandScheduler::GetInstance().Schedule(util::align::drivetrain::AutoAlignToPose(swerveDrive, constraints));
        });
    }
    
    frc2::CommandPtr NomNomModeOn(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel, 
                                  subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                  subsystems::intake::Intake *intake, subsystems::shooter::Articulation *articulation) {    
        return frc2::ParallelCommandGroup(
            Shoot(turret, flywheel, kicker, loader, articulation), 
            commands::main::SetIntakeAngleAndPower(intake, constants::intake::kRollerIntakePower)
        ).ToPtr();
    }
    
    frc2::CommandPtr NomNomModeOff(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker, 
                                   subsystems::indexer::Loader *loader, subsystems::intake::Intake *intake) {
        return frc2::SequentialCommandGroup(
            commands::util::SetFlywheelMotorPower(flywheel, 0.0), 
            commands::util::SetLoaderPower(loader, 0.0),
            commands::util::SetShooterKickerPower(kicker, 0.0),
            commands::util::SetIntakeManual(intake, subsystems::intake::State::RetractIntake, 0.0)
        ).ToPtr();
    }


    frc2::CommandPtr NomNomMode(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel, 
                                subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                subsystems::intake::Intake *intake, subsystems::shooter::Articulation *articulation, bool *nomNomMode) {
        if (*nomNomMode == false) {
            *nomNomMode = true; 
            return NomNomModeOn(turret, flywheel, kicker, loader, intake, articulation); 
        } else {
            *nomNomMode = false;
            return NomNomModeOff(flywheel, kicker, loader, intake);  
        }
    }

    frc2::CommandPtr ToggleShooterState(constants::ShooterState *shooterState) {
        return frc2::InstantCommand([shooterState] () {
            if (*shooterState == constants::ShooterState::Automatic)
            {
                *shooterState = constants::ShooterState::Fixed;
            }
            else 
            {
                *shooterState = constants::ShooterState::Automatic;
            }
        }).ToPtr();
    }

    frc2::CommandPtr ShooterStateAutomatic(constants::ShooterState *shooterState) {
        return frc2::InstantCommand([shooterState] () {
            *shooterState = constants::ShooterState::Automatic;
        }).ToPtr();
    }
    frc2::CommandPtr ShooterStateFixed(constants::ShooterState *shooterState) {
        return frc2::InstantCommand([shooterState] () {
            *shooterState = constants::ShooterState::Fixed;
        }).ToPtr();
    }
    frc2::CommandPtr ShooterStateReset(constants::ShooterState *shooterState) {
        return frc2::InstantCommand([shooterState] () {
            *shooterState = constants::ShooterState::Reset;
        }).ToPtr();
    }

    frc2::CommandPtr ShooterStateManual(constants::ShooterState *shooterState) {
        return frc2::InstantCommand([shooterState] () {
            *shooterState = constants::ShooterState::Manual;
        }).ToPtr();
    }
    

}
