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
#include <frc/DoubleSolenoid.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace ClimberConstants {
    constexpr bool kSoftLimitEnabledForward = true;
    constexpr bool kSoftLimitEnabledReverse = true;
    constexpr double kTopSoftLimit = 0.0; // in rotations
    constexpr double kBottomSoftLimit = 0.0; // in rotations

    constexpr double kExtensionOffset = 0.0; // in rotationse

    constexpr double kClimberPower = 0.85;

    constexpr units::ampere_t kExtensionContinuousCurrentLimit = units::ampere_t {20.0}; // amps
    constexpr units::ampere_t kExtensionPeakCurrentLimit = units::ampere_t {40.0}; // amps
    constexpr units::time::second_t kExtensionPeakCurrentDuration = units::time::second_t {1.0}; // seconds
    // constexpr double kExtensionConversionRotationsToMeters {1.0};

    //! Need to be changed
    constexpr units::turn_t kDefaultMotorTurns = units::turn_t {0};
    constexpr units::turn_t kL1MotorTurns = units::turn_t {120};
    constexpr units::turn_t kL2L3MotorTurns = units::turn_t {100};
    constexpr units::turn_t kMaxTurnTolerance = units::turn_t {10};
    constexpr units::turn_t kMinTurnTolerance = units::turn_t {-10};

    constexpr bool kDisableCurrentLimit = true;

    constexpr bool kExtensionMotorInverted = false;

    constexpr int kLeftClimberMotorID {13};
    constexpr int kRightClimberMotorID {14};
    constexpr int kClimberLimitSwitchID {1};
};