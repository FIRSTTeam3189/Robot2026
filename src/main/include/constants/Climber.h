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

namespace constants {
    namespace climber {
        constexpr int kClimberMotorID {50};
        
        constexpr bool kTopSoftLimitEnabled {false};
        constexpr bool kBottomSoftLimitEnabled {false};
        constexpr units::turn_t kTopSoftLimit {0.0_tr}; //TODO
        constexpr units::turn_t kBottomSoftLimit {0.0_tr}; //TODO

        constexpr double kClimberGearRatio {1.0}; // Find the gear ratio
        // constexpr double kExtensionConversionRotationsToMeters {1.0};
        constexpr double kClimberPowerUp {0.5};
        constexpr double kClimberPowerDown {-0.5};
        constexpr double kResetClimberPower {-1.0};

        constexpr units::ampere_t kExtensionContinuousCurrentLimit = units::ampere_t{35.0};
        constexpr units::ampere_t kExtensionPeakCurrentLimit = units::ampere_t{60.0};
        constexpr units::second_t kExtensionPeakCurrentDuration = 0.1_s; // seconds
        constexpr bool kExtensionEnableCurrentLimit = true;

        constexpr bool kExtensionMotorInverted {false};
        constexpr ctre::phoenix6::signals::NeutralModeValue kExtensionNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};

        
    };
};