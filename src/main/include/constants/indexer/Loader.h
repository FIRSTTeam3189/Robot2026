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

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>


namespace constants {
    namespace indexer {
        namespace loader {
            constexpr int kLoaderID{30};

            constexpr int kLoaderCurrentLimit {45};

            constexpr double kLoadFuelPower {0.50}; 
            constexpr double kReverseRollerPower {-0.5};

            constexpr bool kLoaderMotorInverted {false};

            constexpr units::second_t kLoaderDelay {0.5_s}; 

            constexpr units::second_t kShootDelay {0.25_s}; 
            
            constexpr rev::spark::SparkBaseConfig::IdleMode kRotationNeutralMode {rev::spark::SparkBaseConfig::IdleMode::kBrake};
        };
    };
};