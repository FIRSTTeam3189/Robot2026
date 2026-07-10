#pragma once

#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/math.h>

#include <stdlib.h>

#include <frc/geometry/Rotation2d.h>


/**
 * A class for simulating the gyro inside a swerve drive.
 * This is so that you can also simulate errors such as
 * gyro drift (where the reading gets inaccurate)
 * from heavy collisions or just naturally over time.
 */
namespace simulation::drive {
    class GyroSimulation {
    public:
        GyroSimulation(units::degree_t averageMotionlessDrift30Seconds, double velocityStandardDeviationPercent);

        units::radians_per_second_t GetMeasuredAngularVelocity() const;
        frc::Rotation2d GetGyroReading() const;
        void SetRotation(frc::Rotation2d rotation);
        void SimulationSubTick(units::radians_per_second_t actualAngularVelocity);

    private:
        frc::Rotation2d GetGyroDTheta(units::radians_per_second_t actualAngularVelocity);
        frc::Rotation2d GetCollisionDrift(units::radians_per_second_t actualAngularVelocity);
        frc::Rotation2d GetAmbientDrift();

        units::radians_per_second_t m_previousAngularVelocity{ 0 };
        units::radians_per_second_t m_measuredAngularVelocity{ 0 }; 

        
        units::radians_per_second_squared_t m_driftAccelerationThreshold{ 500 }; /* The threshold of instantaneous angular acceleration at which the chassis is considered to experience an "impact." */
        units::radian_t m_driftImpactCoefficient{ 1 }; /* The amount of drift, in radians, that the gyro experiences as a result of each multiple of the angular acceleration threshold. */
        units::degree_t m_averageMotionlessDrift30Seconds{ 0 };
        double m_velocityStandardDeviationPercent{ 0 };

        frc::Rotation2d m_gyroReading;
    };
}