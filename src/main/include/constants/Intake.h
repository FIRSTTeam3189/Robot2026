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


namespace constants {
    namespace intake {
        constexpr int kRollerMotorID {15};

        constexpr double kRollerIntakePower {0.96}; //-0.90
        constexpr double kRollerReverseIntakePower {-0.96}; //0.90

        // Other roller configs
        constexpr bool kRollerMotorInverted {true};
        constexpr ctre::phoenix6::signals::NeutralModeValue kRollerMotorNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};


        constexpr units::ampere_t kRotationContinuousCurrentLimit = units::ampere_t{20.0};
        constexpr units::ampere_t kRotationPeakCurrentLimit = units::ampere_t{25.0};
        constexpr units::second_t kRotationPeakCurrentDuration = 0.1_s; // seconds
        constexpr bool kRotationEnableCurrentLimit = true;


        // Pneumatics:
        constexpr int kSolenoidModule {23};
        constexpr int kSolenoidForwardChannel {0};
        constexpr int kSolenoidReverseChannel {1};
        constexpr frc::PneumaticsModuleType kSolenoidType {frc::PneumaticsModuleType::REVPH};
        
    };
};

