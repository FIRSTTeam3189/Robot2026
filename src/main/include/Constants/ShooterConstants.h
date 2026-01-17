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
    AlignFerry,
    DefaultAngle
};

//Shooter Constants
namespace ShooterConstants{
    constexpr int kShooterFlywheelID{7};
    constexpr int kShooterLoaderID{8};
    constexpr int kShooterRotationID{9};
    constexpr int kRotationEncoderID{10};

    constexpr bool kSoftLimitEnabledForward{false};
    constexpr bool kSoftLimitEnabledReverse{false};
    constexpr units::turn_t kTopSoftLimit {0.0_tr};
    constexpr units::turn_t kBottomSoftLimit {0.0_tr}; 
    constexpr units::turn_t kExtensionOffset {0.0_tr};

    //default values for flywheel, use SysId to get real values
    constexpr auto kFlywheelKv = 0.023_V / 1_rad_per_s; 
    constexpr auto kFlywheelKa = 0.001_V / 1_rad_per_s_sq;
    constexpr double kFlywheelQelms = 8.0; // qelms. Velocity error tolerance, in radians per second. Decrease this to more heavily penalize state excursion, or make the controller behave more aggressively.
    constexpr double kFlywheelRelms = 12.0; // relms. Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or make the controller less aggressive. 12 is a good starting point because that is the (approximate) maximum voltage of a battery.

    constexpr double kShooterGearRatio {1.0};
    constexpr double kRotationConversionRotationsToDegrees {1.0};
    // constexpr double kRotationConversionDegreesToRotations {1.0 / kRotationConversionRotationsToDegrees};
    constexpr double kShooterPower {1.0};
    constexpr double kResetShooterPower {-1.0};

    constexpr double kSExtension {0.1}; // Static voltage applied to overcome friction
    constexpr double kVExtension {0.115}; // Volts applied per meter per second
    constexpr double kAExtension {0.0}; // Volts applied per meter per second squared
    constexpr double kPExtension {0.4}; // Volts applied per meter error    
    constexpr double kIExtension {0.0}; // Volts applied per meter integral error
    constexpr double kDExtension {0.0}; // Volts applied per meter velocity error
    constexpr double kExtensionConversionRotationsToDegrees {101010.0};

    constexpr units::turns_per_second_t kMotionMagicMaxVelocity {70_tr / 1.0_s}; // rotations per second
    constexpr auto kMotionMagicMaxAcceleration {750_tr / 1.0_s / 1.0_s}; // rotations per second squared
    constexpr auto kMotionMagicMaxJerk {2000_tr / 1.0_s / 1.0_s / 1.0_s}; // rotations per second cubed

    constexpr units::ampere_t kExtensionContinuousCurrentLimit = units::ampere_t{20.0};
    constexpr units::ampere_t kExtensionPeakCurrentLimit = units::ampere_t{25.0};
    constexpr units::second_t kExtensionPeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kExtensionEnableCurrentLimit = true;

    constexpr bool kExtensionMotorInverted {false};
    constexpr ctre::phoenix6::signals::NeutralModeValue kExtensionNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};

    constexpr double kShooterRotationalGearRatio {0.0};
}
