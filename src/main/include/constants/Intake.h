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

namespace constants
{
    namespace intake

    {
        constexpr int kRollerMotorID{15};
        constexpr int kArticulationMotorID{50}; // TODO change later

        constexpr double kRollerIntakePower{0.60};         //-0.90
        constexpr double kRollerReverseIntakePower{-0.60}; // 0.90

        constexpr double kRotationIntakePowerForward {0.2}; 
        constexpr double kRotationIntakePowerReverse {-0.2};

        constexpr double kRotationIntakePowerShotCompression {0.1};

        constexpr units::degree_t kRetractAngle{0.0_deg}; // 0.90
        constexpr units::degree_t kMidAngle {-60.0_deg};
        constexpr units::degree_t kExtendAngle {-120.0_deg}; // 0.90
        constexpr units::degree_t kShakeUpAngle {-20.0_deg}; 
        constexpr units::degree_t kShakeDownAngle {-100.0_deg}; 
        constexpr units::degree_t kAngleThresholdRollers {-100.0_deg}; 
        constexpr units::degree_t kTargetAngleThreshold {8.0_deg}; 

        constexpr units::time::second_t kShakeDelay {0.6_s}; 

        constexpr bool kArticulationMotorInverted{false};

        constexpr units::turns_per_second_t kMotionMagicMaxVelocity{70_tr / 1.0_s}; // rotations per second
        constexpr auto kMotionMagicMaxAcceleration{750_tr / 1.0_s / 1.0_s};         // rotations per second squared
        constexpr auto kMotionMagicMaxJerk{2000_tr / 1.0_s / 1.0_s / 1.0_s};        // rotations per second cubed

        constexpr bool kRollerMotorInverted{false};
        constexpr double kRotationConversionRotationsToDegrees {20.0}; 
        constexpr ctre::phoenix6::signals::NeutralModeValue kRollerMotorNeutralMode{ctre::phoenix6::signals::NeutralModeValue::Brake};
        constexpr ctre::phoenix6::signals::NeutralModeValue kRotationNeutralMode{ctre::phoenix6::signals::NeutralModeValue::Brake};

        constexpr bool kSoftLimitEnabledForward = false;
        constexpr bool kSoftLimitEnabledReverse = false;

        constexpr units::turn_t kTopSoftLimit = 0.0_tr;
        constexpr units::turn_t kBottomSoftLimit = 0.0_tr; 

        constexpr units::ampere_t kRotationContinuousCurrentLimit = units::ampere_t{20.0};
        constexpr units::ampere_t kRotationPeakCurrentLimit = units::ampere_t{25.0};
        constexpr units::second_t kRotationPeakCurrentDuration = 0.1_s; // seconds
        constexpr bool kRotationEnableCurrentLimit = true;

        constexpr double kArticulationKS = 0.2;
        constexpr double kArticulationKV = 0.15;
        constexpr double kArticulationKI = 0.0;
        constexpr double kArticulationKD = 0.0;
        constexpr double kArticulationKG = 0.65;
        constexpr double kArticulationKA = 0.0;
        constexpr double kArticulationKP = 0.225;


        constexpr units::second_t kRollerDelay {1.0_s}; 


    };
};
