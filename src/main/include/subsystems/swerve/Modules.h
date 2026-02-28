#pragma once 

#include <frc/geometry/Translation2d.h>
#include "util/SwerveModule.h"

namespace subsystems {
    namespace swerve {
        struct Modules {
            frc::Translation2d m_frontLeftLocation;
            frc::Translation2d m_frontRightLocation;
            frc::Translation2d m_backLeftLocation;
            frc::Translation2d m_backRightLocation;

            SwerveModule m_frontLeft;
            SwerveModule m_frontRight;
            SwerveModule m_backLeft;
            SwerveModule m_backRight;
        };
    };
}; 