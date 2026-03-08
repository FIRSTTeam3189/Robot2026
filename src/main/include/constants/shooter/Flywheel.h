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
    namespace shooter {
        enum class FlywheelState { AutoVelocity, FixedVelocity, ManualVelocity, ZeroVelocity};
        namespace flywheel {
            constexpr int kShooterFlywheelID{24};

            constexpr double kAutoStaticFlywheelPower {-0.5};
            constexpr double kFlywheelFerryPower {-0.95}; 
            constexpr double kFlywheelShootPower {-0.90};

            constexpr double kFlywheelMaxRPM {4100.0}; 

            constexpr double kMotorConversionRPMToRPS {60.0}; 

            constexpr units::turns_per_second_t kFlywheelIncrementVelocity {1.667_tps}; 
            constexpr units::turns_per_second_t kFlywheelDecrementVelocity {-1.667_tps}; 


            constexpr units::turns_per_second_t kFlywheelFerryVelocity {40_tps}; 
            constexpr units::turns_per_second_t kFlywheelShootVelocity {0_tps}; // fixed state actually

            constexpr units::turns_per_second_t kFlywheelFixedVelocity {100_tps}; 

            constexpr units::second_t kFlywheelRunDelay {0.3_s}; 
            
            constexpr bool kFlywheelMotorInverted {false};
            constexpr ctre::phoenix6::signals::NeutralModeValue kFlywheelNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Coast};

            // flywheel pid stuff
            constexpr double kS {0.0};
            constexpr double kV {0.12};
            constexpr double kP {1.0};
            constexpr double kI {0.0};
            constexpr double kD {0.0};

            constexpr units::volt_t kFeedForwardVoltage {0.5_V};

            // flywheel regression constants
            // A is just a name of the constant, the number tells you the degree of that term 
            constexpr double kA4 {70.18}; 
            constexpr double kA3 {-887.7}; 
            constexpr double kA2 {4075.0}; 
            constexpr double kA1 {-7705}; 
            constexpr double kA0 {7294}; 
            
        };
    };
};
