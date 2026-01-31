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

//Turret Constants
namespace LoaderConstants {
    constexpr int kShooterLoaderID{21};

    constexpr double kLoaderMotorPower {0.75}; 

    constexpr bool kLoaderMotorInverted {true};
    constexpr ctre::phoenix6::signals::NeutralModeValue kLoaderNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};

}
