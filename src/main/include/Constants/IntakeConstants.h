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

namespace IntakeConstants{
    constexpr int kRollerMotorID {15};

    constexpr double kRollerIntakePower {-0.90};
    constexpr double kRollerReverseIntakePower {0.90};

    // Other roller configs
    constexpr bool kRollerMotorInverted {true};
    constexpr rev::spark::SparkBaseConfig::IdleMode kRollerMotorIdleMode {rev::spark::SparkBaseConfig::IdleMode::kBrake};
    constexpr unsigned int kRollerMotorCurrentLimit {45};


    // Pneumatics:
    constexpr int kSolenoidModule {23};
    constexpr int kSolenoidForwardChannel {1};
    constexpr int kSolenoidReverseChannel {0};
    constexpr frc::PneumaticsModuleType kSolenoidType {frc::PneumaticsModuleType::REVPH};
    
}
