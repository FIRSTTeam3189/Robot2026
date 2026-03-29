#include "actions/Robot.h"

namespace actions::robot
{
    frc2::SequentialCommandGroup Shoot(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel,
                                       subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, subsystems::shooter::Hood *articulation)
    {
        return frc2::SequentialCommandGroup(
            // SetTurretAngle(turret, TurretTarget::AlignHub),
            // SetHoodAngle(articulation, HoodTarget::),
            frc2::ParallelCommandGroup(
                frc2::SequentialCommandGroup(
                    commands::util::SetFlywheelMotorVelocity(flywheel, flywheel->GetFlywheelTargetRPM()), 
                    commands::util::SetLoaderPower(loader, -constants::indexer::loader::kLoadFuelPower)),
                    commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)
                )
            );
    }

    frc2::CommandPtr ShootEnd(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker,
                              subsystems::indexer::Loader *loader, subsystems::shooter::Hood *articulation)
    {
        return frc2::SequentialCommandGroup(
            commands::util::SetFlywheelMotorVelocity(flywheel, 0.0),
            commands::util::SetLoaderPower(loader, 0.0),
            commands::util::SetShooterKickerPower(kicker, 0.0),
            commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees0)
        ).ToPtr();
    }

    frc2::CommandPtr ShootWithVelocity(subsystems::shooter::Flywheel *flywheel, double rpm)
    {
        return commands::util::SetFlywheelMotorVelocity(flywheel, rpm).ToPtr();
    }

    // ! timer shoot 
    frc2::ParallelCommandGroup AdvancedShoot(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel,
                                             subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                             subsystems::shooter::Hood *articulation, subsystems::intake::Intake *intake)
    {
        return frc2::ParallelCommandGroup(
            // actions::intake::ShakeIntake(intake),
            frc2::SequentialCommandGroup(
                frc2::WaitCommand(constants::indexer::loader::kShootDelay),
                // commands::util::SetLoaderPower(loader, -constants::indexer::loader::kLoadFuelPower),
                // commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)
                frc2::ParallelCommandGroup(
                    commands::main::OscillateLoaderPower(loader, -constants::indexer::loader::kLoadFuelPower),
                    commands::main::OscillateKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)
                )
            ),
            commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::AligntoHub),
            commands::util::SetFlywheelMotorVelocity(flywheel, flywheel->GetFlywheelTargetRPM())
        );
    }

    // // ! timer shoot
    frc2::CommandPtr AdvancedShootEnd(subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader,
                                      subsystems::shooter::Hood *articulation, subsystems::shooter::Flywheel *flywheel,
                                      subsystems::intake::Intake *intake)
    {
        return frc2::SequentialCommandGroup(
            commands::util::SetLoaderPower(loader, 0.0),
            commands::util::SetShooterKickerPower(kicker, 0.0),
            actions::shooter::flywheel::ResetFlywheelVelocity(flywheel), 
            commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees0)
        ).ToPtr();
    }

    frc2::SequentialCommandGroup AdvancedShootSlowSwerves(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel,
                                                        subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, 
                                                        subsystems::shooter::Hood *articulation, subsystems::intake::Intake *intake,
                                                        subsystems::swerve::Drive *drive, subsystems::swerve::State *driveState,
                                                        frc::PS5Controller *bill)
    {
        return frc2::SequentialCommandGroup(
            // actions::swerve::SlowDownSwerves(drive, driveState, bill),
            frc2::ParallelCommandGroup(
                actions::intake::ShakeIntake(intake),
                frc2::SequentialCommandGroup(
                    frc2::WaitCommand(constants::indexer::loader::kShootDelay),
                    // commands::util::SetLoaderPower(loader, -constants::indexer::loader::kLoadFuelPower),
                    // commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)
                    frc2::ParallelCommandGroup(
                        commands::main::OscillateLoaderPower(loader, -constants::indexer::loader::kLoadFuelPower),
                        commands::main::OscillateKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)
                    )
                ),
                commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::AligntoHub),
                commands::util::SetFlywheelMotorVelocity(flywheel, flywheel->GetFlywheelTargetRPM())
            ) 
        );
    }

    frc2::CommandPtr AdvancedShootSlowSwervesEnd(subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader,
                                                 subsystems::shooter::Hood *articulation, subsystems::shooter::Flywheel *flywheel,
                                                 subsystems::intake::Intake *intake, subsystems::swerve::Drive *drive, 
                                                 subsystems::swerve::State *driveState, frc::PS5Controller *bill)
    {
        return frc2::SequentialCommandGroup(
            actions::shooter::flywheel::ResetFlywheelVelocity(flywheel), 
            // actions::swerve::DefaultSwerveSpeed(drive, driveState, bill),
            commands::util::SetLoaderPower(loader, 0.0),
            commands::util::SetShooterKickerPower(kicker, 0.0),
            commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees0),
            actions::intake::IntakeRetractState(intake)
        ).ToPtr();
    }

    // frc2::ParallelCommandGroup AdvancedShootWithIntakeCompression(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel,
    //     subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader, subsystems::shooter::Hood *articulation, subsystems::intake::Intake *intake)
    //     {
    //     return frc2::ParallelCommandGroup(
    //         frc2::SequentialCommandGroup(
    //             frc2::WaitCommand(constants::indexer::loader::kShootDelay),
    //             commands::util::SetLoaderPower(loader, constants::indexer::loader::kLoadFuelPower),
    //             commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPower)),
    //         commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::AligntoHub),
    //         commands::util::SetIntakeRotationPower(intake, constants::intake::kRotationIntakePowerShotCompression));
    // }
        
    // frc2::CommandPtr AdvancedShootEndWithIntakeCompression(subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader,
    //     subsystems::shooter::Hood *articulation, subsystems::intake::Intake *intake)
    //     {
    //     return frc2::SequentialCommandGroup(
    //                commands::util::SetLoaderPower(loader, 0.0),
    //                commands::util::SetShooterKickerPower(kicker, 0.0),
    //                commands::util::SetHoodAngle(articulation, constants::shooter::HoodTarget::Degrees0),
    //                commands::util::SetIntakeState(intake, constants::intake::kRollerIntakePower)

    //                    )
    //         .ToPtr();
    // }
    
    frc2::CommandPtr EjectFuel(subsystems::intake::Intake *intake, subsystems::indexer::Loader *loader, 
                                subsystems::indexer::Kicker *kicker) {
        return frc2::SequentialCommandGroup(
            commands::util::SetIntakeRollerMotor(intake, constants::intake::kRollerReverseIntakePower),
            commands::util::SetLoaderPower(loader, constants::indexer::loader::kReverseRollerPower),
            commands::util::SetShooterKickerPower(kicker, constants::indexer::kicker::kKickerMotorPowerReverse)
        ).ToPtr(); 
    }

    frc2::CommandPtr EjectFuelEnd(subsystems::intake::Intake *intake, subsystems::indexer::Loader *loader, 
                                subsystems::indexer::Kicker *kicker) {
        return frc2::SequentialCommandGroup(
            commands::util::SetIntakeRollerMotor(intake, constants::intake::kRollerIntakePower),
            commands::util::SetLoaderPower(loader, 0.0),
            commands::util::SetShooterKickerPower(kicker, 0.0)
        ).ToPtr(); 
    }

    frc2::CommandPtr GoToClosestPose(subsystems::swerve::Drive *swerveDrive, pathplanner::PathConstraints constraints)
    {
        return frc2::cmd::RunOnce([swerveDrive, constraints]()
                                  { frc2::CommandScheduler::GetInstance().Schedule(util::align::drivetrain::AutoAlignToPose(swerveDrive, constraints)); });
    }

    frc2::CommandPtr NomNomModeOn(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel,
                                  subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader,
                                  subsystems::intake::Intake *intake, subsystems::shooter::Hood *articulation)
    {
        return frc2::ParallelCommandGroup(
                   Shoot(turret, flywheel, kicker, loader, articulation),
                   commands::main::SetIntakeAngleAndPower(intake, constants::intake::kRollerIntakePower))
            .ToPtr();
    }

    frc2::CommandPtr NomNomModeOff(subsystems::shooter::Flywheel *flywheel, subsystems::indexer::Kicker *kicker,
                                   subsystems::indexer::Loader *loader, subsystems::intake::Intake *intake)
    {
        return frc2::SequentialCommandGroup(
                   commands::util::SetFlywheelMotorPower(flywheel, 0.0),
                   commands::util::SetLoaderPower(loader, 0.0),
                   commands::util::SetShooterKickerPower(kicker, 0.0),
                   commands::util::SetIntakeState(intake, 0.0))
            .ToPtr();
    }

    frc2::CommandPtr NomNomMode(subsystems::shooter::Turret *turret, subsystems::shooter::Flywheel *flywheel,
                                subsystems::indexer::Kicker *kicker, subsystems::indexer::Loader *loader,
                                subsystems::intake::Intake *intake, subsystems::shooter::Hood *articulation, bool *nomNomMode)
    {
        if (*nomNomMode == false)
        {
            *nomNomMode = true;
            return NomNomModeOn(turret, flywheel, kicker, loader, intake, articulation);
        }
        else
        {
            *nomNomMode = false;
            return NomNomModeOff(flywheel, kicker, loader, intake);
        }
    }

    frc2::CommandPtr ToggleShooterState(constants::ShooterState *shooterState)
    {
        return frc2::InstantCommand([shooterState]()
                                    {
            if (*shooterState == constants::ShooterState::Automatic)
            {
                *shooterState = constants::ShooterState::Fixed;
            }
            else 
            {
                *shooterState = constants::ShooterState::Automatic;
            } })
            .ToPtr();
    }

    frc2::CommandPtr ShooterStateAutomatic(constants::ShooterState *shooterState)
    {
        return frc2::InstantCommand([shooterState]()
                                    { *shooterState = constants::ShooterState::Automatic; })
            .ToPtr();
    }
    frc2::CommandPtr ShooterStateFixed(constants::ShooterState *shooterState)
    {
        return frc2::InstantCommand([shooterState]()
                                    { *shooterState = constants::ShooterState::Fixed; })
            .ToPtr();
    }
    frc2::CommandPtr ShooterStateReset(constants::ShooterState *shooterState)
    {
        return frc2::InstantCommand([shooterState]()
                                    { *shooterState = constants::ShooterState::Reset; })
            .ToPtr();
    }

    frc2::CommandPtr ShooterStateManual(constants::ShooterState *shooterState)
    {
        return frc2::InstantCommand([shooterState]()
                                    { *shooterState = constants::ShooterState::Manual; })
            .ToPtr();
    }

    frc2::CommandPtr ShooterStateHoodAndInterpAuto(constants::ShooterState *shooterState)
    {
        return frc2::InstantCommand([shooterState]()
                                    { *shooterState = constants::ShooterState::HoodAndInterpAuto; })
            .ToPtr();
    }

}
