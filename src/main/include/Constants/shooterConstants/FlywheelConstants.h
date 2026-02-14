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
namespace FlywheelConstants {
    constexpr int kShooterFlywheelID{24};

    constexpr double kFlyWheelPowerIncrement {0.05}; 
    constexpr double kFlyWheelPowerDecrement {-0.05}; 

    constexpr units::second_t kFlywheelRunDelay {0.75_s}; 
    
    constexpr bool kFlywheelMotorInverted {false};
    constexpr ctre::phoenix6::signals::NeutralModeValue kFlywheelNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Coast};

}
