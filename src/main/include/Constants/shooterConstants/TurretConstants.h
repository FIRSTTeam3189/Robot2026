#pragma once

#include <iostream>
#include <units/time.h>
#include <units/frequency.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>

enum class TurretTarget {
    AlignHub,
    PointStraight,
    Point45Deg,
    Point135Deg,
};

//Turret Constants
namespace TurretConstants {
    constexpr int kShooterRotationID{20};

    constexpr double kRotationAngleTolerance {1.0};

    constexpr bool kSoftLimitEnabledForward{false};
    constexpr bool kSoftLimitEnabledReverse{false};
    constexpr units::turn_t kTopSoftLimit {0.0_tr};
    constexpr units::turn_t kBottomSoftLimit {0.0_tr}; 
    constexpr units::turn_t kRotationOffset {0.0_tr};

    constexpr double kShooterGearRatio {1.0};

    constexpr double kRotationConversionRotationsToDegrees {1.0/0.3571428571 * 2};

    //! for v1
    // constexpr double kRotationConversionRotationsToDegrees {4.0};
    
    
    //constexpr double kRotationConversionRotationsToDegrees {(1/(((264/48) * (5)) / 360))}

    // constexpr double kRotationConversionDegreesToRotations {1.0 / kRotationConversionRotationsToDegrees};


    // constexpr double kSRotation {0.03}; // Static voltage applied to overcome friction
    // constexpr double kVRotation {0.15}; // Volts applied per meter per second
    // constexpr double kARotation {0.0}; // Volts applied per meter per second squared
    // constexpr double kPRotation {0.7}; // Volts applied per meter error    
    // constexpr double kIRotation {0.0}; // Volts applied per meter integral error
    // constexpr double kDRotation {0.0}; // Volts applied per meter velocity error

    constexpr double kSRotation {0.10724}; // Static voltage applied to overcome friction
    constexpr double kVRotation {0.11365}; // Volts applied per meter per second
    constexpr double kARotation {0.0013311}; // Volts applied per meter per second squared
    constexpr double kPRotation {-5.0104}; // Volts applied per meter error    
    constexpr double kIRotation {0.0}; // Volts applied per meter integral error
    constexpr double kDRotation {0.0082091}; // Volts applied per meter velocity error

    constexpr units::turns_per_second_t kMotionMagicMaxVelocity {70_tr / 1.0_s}; // rotations per second
    constexpr auto kMotionMagicMaxAcceleration {750_tr / 1.0_s / 1.0_s}; // rotations per second squared
    constexpr auto kMotionMagicMaxJerk {2000_tr / 1.0_s / 1.0_s / 1.0_s}; // rotations per second cubed

    constexpr units::ampere_t kRotationContinuousCurrentLimit = units::ampere_t{20.0};
    constexpr units::ampere_t kRotationPeakCurrentLimit = units::ampere_t{25.0};
    constexpr units::second_t kRotationPeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kRotationEnableCurrentLimit = true;

    constexpr bool kRotationMotorInverted {true};
    constexpr ctre::phoenix6::signals::NeutralModeValue kRotationNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};

}
