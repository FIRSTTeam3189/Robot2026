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
        enum ArticulationTarget {
            AligntoHub,
            Stationary,
            Manual,
            FixedAlign,
            Degrees0,
            Degrees20,
            Degrees25,
            Degrees30,
            Degrees40
        }; 
        //Articulation Constants

        namespace articulation {

            constexpr int kArticulationMotorID{21};

            constexpr double kRotationAngleTolerance {1.0};
            constexpr bool kSoftLimitEnabledForward{false};
            constexpr bool kSoftLimitEnabledReverse{false};

            constexpr units::turn_t kTopSoftLimit {0.0_tr};
            constexpr units::turn_t kBottomSoftLimit {0.0_tr}; 
            constexpr units::turn_t kRotationOffset {0.0_tr};

            constexpr units::degree_t kArticulationTolerance {2.0_deg};

            //default values for articulation, use SysId to get real values

            constexpr auto kArticulationKv = 0.023_V / 1_rad_per_s; 
            constexpr auto kArticulationlKa = 0.001_V / 1_rad_per_s_sq;


            constexpr double kShooterGearRatio {1.0};

            // 1 rotation of the motor is 18 degrees 
            constexpr double kRotationConversionRotationsToDegrees {18.0};

            // constexpr double kRotationConversionRotationsToDegrees {4.0};
            // constexpr double kRotationConversionDegreesToRotations {1.0 / kRotationConversionRotationsToDegrees};

            constexpr double kShooterPower {1.0};
            constexpr double kResetShooterPower {-1.0};


            constexpr double kSRotation {0.375}; // Static voltage applied to overcome friction
            constexpr double kVRotation {0.6}; // Volts applied per meter per second
            constexpr double kARotation {0.0}; // Volts applied per meter per second squared
            constexpr double kPRotation {3.5}; // Volts applied per meter error    
            constexpr double kIRotation {0.0}; // Volts applied per meter integral error
            constexpr double kDRotation {0.0}; // Volts applied per meter velocity error


            constexpr units::turns_per_second_t kMotionMagicMaxVelocity {70_tr / 1.0_s}; // rotations per second
            constexpr auto kMotionMagicMaxAcceleration {750_tr / 1.0_s / 1.0_s}; // rotations per second squared
            constexpr auto kMotionMagicMaxJerk {2000_tr / 1.0_s / 1.0_s / 1.0_s}; // rotations per second cubed


            constexpr units::ampere_t kRotationContinuousCurrentLimit = units::ampere_t{20.0};
            constexpr units::ampere_t kRotationPeakCurrentLimit = units::ampere_t{25.0};
            constexpr units::second_t kRotationPeakCurrentDuration = 0.1_s; // seconds

            constexpr bool kRotationEnableCurrentLimit = true;
            constexpr bool kRotationMotorInverted {true};

            constexpr ctre::phoenix6::signals::NeutralModeValue kRotationNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};

            constexpr double kShooterRotationalGearRatio {0.0};

            constexpr units::degree_t kHoodStationaryAngle {15.0_deg};
            constexpr units::degree_t kHoodMaxAngle {50.0_deg}; 

            constexpr units::degree_t kArticulationAngleIncrement {5.0_deg};
            constexpr units::degree_t kArticulationAngleDecrement {-5.0_deg};

        };
    };
};

