#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>


namespace constants::simulation::swerve_drive {
    // Gyro constants
    constexpr static bool kEnableGyroDrifting{ true }; //! Currently not implemented
    constexpr static units::radians_per_second_squared_t kGyroDriftAccelerationThreshold{ 500 };
    constexpr static units::degree_t kGyroAverageDriftingIn30SecondsMotionless{ 0.5 };
    constexpr static units::radian_t kGyroDriftCoefficient{ 1 };
    constexpr static double kGyroVelocityMeasurementStandardDeviationPercent{ 0.02 }; 
}