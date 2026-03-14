#pragma once 

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "util/align/Drivetrain.h"
#include "util/interpolator/Data.h"
#include "util/align/Turret.h"

#include "subsystems/shooter/Hood.h"
#include "subsystems/shooter/Flywheel.h"
#include "subsystems/swerve/Drive.h"


#include "constants/shooter/Turret.h"

namespace actions {
    namespace interpolator {  
        frc2::CommandPtr AddInterpolatorPoint(util::interpolator::Data  *interpolatorData, subsystems::swerve::Drive *swerveDrive, 
                                            subsystems::shooter::Flywheel *flywheel, subsystems::shooter::Hood *articulation); 
        
        frc2::CommandPtr SaveAllInterpolationPoints(util::interpolator::Data  *interpolatorData);
        frc2::CommandPtr RemoveLastInterpolationPoint(util::interpolator::Data  *interpolatorData);
    };
};