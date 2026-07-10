#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/angle.h>
#include <units/force.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/math.h>
#include <random>
#include <numbers>

#include "simulation/box2d/box2d.h"

namespace simulation::commonmath {
    inline std::random_device randomNumberGenerator;
    // Geometry converting

    inline frc::Translation2d GetTranslationalSpeeds(const frc::ChassisSpeeds& speeds) {
        return frc::Translation2d{
            units::meter_t{ speeds.vx.value() },
            units::meter_t{ speeds.vy.value() }
        };
    }

    inline units::radian_t GetVectorDirection(const b2Vec2& vector) {
        return units::radian_t{ std::atan2(vector.y, vector.x) };
    }

    // Type converting 

    inline b2Vec2 ToBox2dVector2(const frc::Translation2d& translation) {
        return b2Vec2{ 
            (float)translation.X().value(), 
            (float)translation.Y().value() 
        };
    }

    inline frc::Translation2d ToTranslation2d(const b2Vec2& vector) {
        return frc::Translation2d{ 
            units::meter_t{ vector.x }, 
            units::meter_t{ vector.y } 
        };
    }
    
    inline frc::Rotation2d GetAngle(const frc::Translation2d& translation) {
        const double tooSmall = 1e-6;
        
        return translation.Norm().value() < tooSmall ? frc::Rotation2d{ 0_deg } : translation.Angle();
    }

    /** 
     * Converts polar coordinates, which take in a distance (magnitude)
     * and a specified angle, to cartesian coordinates (x and y)
     */
    inline b2Vec2 PolarToCartesian(double magnitude, double angleRadians) {
        return b2Vec2{
            (float)(magnitude * std::cos(angleRadians)),
            (float)(magnitude * std::sin(angleRadians))
        };
    }
    
    /** 
     * Converts polar coordinates, which take in a distance (magnitude)
     * and a specified angle, to cartesian coordinates (x and y)
     */
    inline b2Vec2 PolarToCartesian(units::newton_t magnitude, units::radian_t angle) {
        return PolarToCartesian((float)magnitude.value(), (float)angle.value());
    }

    /**
     * Projects the swerve module speed onto the direction of the current swerve facing.
     */
    inline units::meters_per_second_t ProjectSwerveState(frc::SwerveModuleState swerveSpeed, frc::Rotation2d currentFacing) {
        // Get the angle of the swerve module's current direction
        frc::Rotation2d swerveModuleAngle = swerveSpeed.angle;

        // Calculate the cosine of the angle difference between swerve 
        // module direction and the desired direction
        units::radian_t cosTheta{ units::math::cos(swerveModuleAngle.Radians() - currentFacing.Radians()) };

        // Scale the speed by the cosine value to get the projection
        return swerveSpeed.speed * cosTheta.value();
    }

    /**
     * Generates a random normal with a given mean and standard deviation
     * of/from the distribution
     */
    inline double GenerateRandomNormal(double mean, double standardDeviation) {
        std::uniform_real_distribution<double> uniform(0.0, 1.0);

        double u1 = uniform(randomNumberGenerator);
        double u2 = uniform(randomNumberGenerator);
        // Box–Muller transform https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
        double z0 = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
        
        return z0 * standardDeviation + mean;
    };
}