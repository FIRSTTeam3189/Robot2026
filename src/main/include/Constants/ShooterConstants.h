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

    constexpr double kShooterGearRatio {1.0};
    constexpr double kExtensionConversionRotationsToMeters {1.0};
    constexpr double kShooterPower {1.0};
    constexpr double kResetShooterPower {-1.0};

    constexpr units::ampere_t kExtensionContinuousCurrentLimit = units::ampere_t{20.0};
    constexpr units::ampere_t kExtensionPeakCurrentLimit = units::ampere_t{25.0};
    constexpr units::second_t kExtensionPeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kExtensionEnableCurrentLimit = true;

    constexpr bool kExtensionMotorInverted {false};
    constexpr ctre::phoenix6::signals::NeutralModeValue kExtensionNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};
}
