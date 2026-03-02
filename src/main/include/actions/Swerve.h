#pragma once 

#include <frc/PS5Controller.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "commands/main/Drive.h"

#include "subsystems/swerve/Drive.h" 


namespace actions {
    namespace swerve {
        frc2::InstantCommand SlowDownSwerves(subsystems::swerve::Drive *swerveDrive, subsystems::swerve::State *driveState, frc::PS5Controller *bill);
        frc2::InstantCommand DefaultSwerveSpeed(subsystems::swerve::Drive *swerveDrive, subsystems::swerve::State *driveState, frc::PS5Controller *bill);
        frc2::CommandPtr ClimbAlignSwerves(subsystems::swerve::Drive *swerveDrive, subsystems::swerve::State *driveState, frc::PS5Controller *bill);
    };
};