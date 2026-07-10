#include "simulation/drive/GyroSimulation.h"
#include "simulation/CommonMath.h"

#include "constants/simulation/SimulatedArena.h"
#include "constants/simulation/SwerveDrive.h"

namespace simulation::drive {
    // Public
    GyroSimulation::GyroSimulation(units::degree_t averageMotionlessDrift30Seconds, double velocityStandardDeviationPercent) 
    : m_averageMotionlessDrift30Seconds(averageMotionlessDrift30Seconds)
    , m_velocityStandardDeviationPercent(velocityStandardDeviationPercent)
    {

    }

    /**
     * Returns the measured angular velocity of the gyro. This includes
     * measurement errors from drifting or no-motion.
     */
    units::radians_per_second_t GyroSimulation::GetMeasuredAngularVelocity() const {
        return m_measuredAngularVelocity;
    }

    /**
     * Returns the estimated rotation of the gyro. This includes
     * measurement errors from drifting or no-motion.
     */
    frc::Rotation2d GyroSimulation::GetGyroReading() const{
        return m_gyroReading;
    }

    /**
     * Calibrates the gyro rotation to the specified rotation.
     */
    void GyroSimulation::SetRotation(frc::Rotation2d rotation) {
        m_gyroReading = rotation;
    }

    /**
     * Updates the gyro simulation.
     */
    void GyroSimulation::SimulationSubTick(units::radians_per_second_t actualAngularVelocity) {
        const frc::Rotation2d collisionDrift = GetCollisionDrift(actualAngularVelocity);
        m_gyroReading = m_gyroReading + collisionDrift;

        const frc::Rotation2d angleDelta = GetGyroDTheta(actualAngularVelocity);
        m_gyroReading = m_gyroReading + angleDelta;

        if (constants::simulation::swerve_drive::kEnableGyroDrifting) {
            const frc::Rotation2d noMotionDrift = GetAmbientDrift();
            m_gyroReading = m_gyroReading + noMotionDrift;
        }
    }

    // Private

    frc::Rotation2d GyroSimulation::GetCollisionDrift(units::radians_per_second_t actualAngularVelocity) {
        units::time::second_t simulationDeltaTime = constants::simulation::simulated_arena::kSubDeltaTime;
        
        // Get how fast the robot is moving in rotation
        units::radians_per_second_squared_t angularAcceleration = 
            (actualAngularVelocity - m_previousAngularVelocity) / simulationDeltaTime;

         // Only apply drift when acceleration exceeds the impact threshold
        units::radian_t driftMagnitude{ 0 };
        if (units::math::abs(angularAcceleration) > m_driftAccelerationThreshold) {
            driftMagnitude =
                units::radian_t{(units::math::abs(angularAcceleration) / m_driftAccelerationThreshold)
                * m_driftImpactCoefficient};
        }

        // Drift direction is opposite to the acceleration direction
        units::radian_t drift{std::copysign(driftMagnitude.value(), -angularAcceleration.value())};

        m_previousAngularVelocity = actualAngularVelocity;

        return frc::Rotation2d{ drift };
    }


    /**
     * Generates ambient drift.
     * This siimulates the minor drifting of the gyro that
     * occurs regardless of whether the robot is
     * moving or not.
     */
    frc::Rotation2d GyroSimulation::GetAmbientDrift() {
        units::degree_t averageDriftingInOnePeriod{
            m_averageMotionlessDrift30Seconds / 30 
            * constants::simulation::simulated_arena::kSubDeltaTime.value()
        };

        units::degree_t driftingInThisPeriod{
            simulation::commonmath::GenerateRandomNormal(0, averageDriftingInOnePeriod.value())
        };

        return frc::Rotation2d(driftingInThisPeriod);
    }


    frc::Rotation2d GyroSimulation::GetGyroDTheta(units::radians_per_second_t actualAngularVelocity) {
        m_measuredAngularVelocity = units::radians_per_second_t{ 
            simulation::commonmath::GenerateRandomNormal(
                actualAngularVelocity.value(),
                m_velocityStandardDeviationPercent
                * units::math::abs(actualAngularVelocity).value()
            )
        };

        units::radian_t dTheta {
            m_measuredAngularVelocity.value() 
            * constants::simulation::simulated_arena::kSubDeltaTime.value()
        };

        return frc::Rotation2d(dTheta);
    }
}