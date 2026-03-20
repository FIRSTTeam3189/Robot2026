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
    namespace indexer {
        namespace kicker {
            constexpr int kShooterKickerID{22};
            constexpr double kKickerMotorPower {0.60}; 
            constexpr double kKickerMotorPowerReverse {0.75}; 
            
            constexpr bool kKickerMotorInverted {true};
            constexpr ctre::phoenix6::signals::NeutralModeValue kKickerNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};
        };
    };
};
