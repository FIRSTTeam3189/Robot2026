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

#define PI 3.14159265358979323846

namespace IntakeConstants {
    constexpr int kIntakeMotorID {19};
    constexpr bool kExtensionMotorInverted {false};
    constexpr ctre::phoenix6::signals::NeutralModeValue kExtensionNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};

    
}