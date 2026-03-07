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
            constexpr int kShooterFlywheel2ID {1000}; //! Change later

            constexpr double kAutoStaticFlywheelPower {-0.5};
            constexpr double kFlywheelFerryPower {-0.95}; 
            constexpr double kFlywheelShootPower {-0.90};

            constexpr double kFlywheelMaxRPM {4100.0}; 

            constexpr double kMotorConversionRPMToRPS {60.0}; 

            constexpr double kFlywheelIncrementVelocity {300}; 
            constexpr double kFlywheelDecrementVelocity {-300}; 


            constexpr double kFlywheelFerryVelocity {2400}; 
            constexpr double kFlywheelShootVelocity {0}; // fixed state actually

            constexpr double kFlywheelFixedVelocity {6000}; 

            constexpr units::second_t kFlywheelRunDelay {0.3_s}; 
            
            constexpr bool kFlywheelMotorInverted {false};
            constexpr bool kInvertSecondMotor {true};
            constexpr ctre::phoenix6::signals::NeutralModeValue kFlywheelNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Coast};

            // flywheel pid stuff
            constexpr double kS {0.0};
            constexpr double kV {0.12};
            constexpr double kP {1.0};
            constexpr double kI {0.0};
            constexpr double kD {0.0};

            constexpr units::volt_t kFeedForwardVoltage {0.5_V};
        };
    };
};
