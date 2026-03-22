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
        enum class TurretTarget {
            AlignHub,
            Point0Deg,
            PointStraight,
            Point45Deg,
            Point135Deg,
            PointNeg56Deg,
            PointMaxDeg
        };

        //Turret Constants
        namespace turret {
            constexpr int kShooterRotationID{31};

            constexpr double kRotationAngleTolerance {1.0};

            constexpr bool kSoftLimitEnabledForward{true};
            constexpr bool kSoftLimitEnabledReverse{true};
            constexpr units::turn_t kTopSoftLimit {18.0_tr};
            constexpr units::turn_t kBottomSoftLimit {-4.27_tr}; 
            constexpr units::turn_t kRotationOffset {0.0_tr};

            constexpr double kShooterGearRatio {1.0};

            // V0 gear ratio 
            // constexpr double kRotationConversionRotationsToDegrees {1.0/0.3571428571 * 2};
            

            //! for v1
            // constexpr double kRotationConversionRotationsToDegrees {4.0};
            
            // V1 
            constexpr double kRotationConversionRotationsToDegrees {(360 / ( 5.5 * 5 ))}; // 5.5 is num big gear / num small gear, 5 is internal ratio, and *2 idk gg

            // constexpr double kRotationConversionDegreesToRotations {1.0 / kRotationConversionRotationsToDegrees};


            constexpr double kSRotation {0.0}; // Static voltage applied to overcome friction
            constexpr double kVRotation {0.16}; // Volts applied per meter per second
            constexpr double kARotation {0.0}; // Volts applied per meter per second squared
            constexpr double kPRotation {2.25}; // Volts applied per meter error    
            constexpr double kIRotation {0.0}; // Volts applied per meter integral error
            constexpr double kDRotation {0.0}; // Volts applied per meter velocity error

            constexpr units::turns_per_second_t kMotionMagicMaxVelocity {70_tr / 1.0_s}; // rotations per second
            constexpr auto kMotionMagicMaxAcceleration {750_tr / 1.0_s / 1.0_s}; // rotations per second squared
            constexpr auto kMotionMagicMaxJerk {2000_tr / 1.0_s / 1.0_s / 1.0_s}; // rotations per second cubed

            constexpr units::ampere_t kRotationContinuousCurrentLimit = units::ampere_t{20.0};
            constexpr units::ampere_t kRotationPeakCurrentLimit = units::ampere_t{25.0};
            constexpr units::second_t kRotationPeakCurrentDuration = 0.1_s; // seconds
            constexpr bool kRotationEnableCurrentLimit = true;

            constexpr units::degree_t kTotalROM {300.0_deg}; // in - range
            constexpr units::degree_t kMinimumAngleNegative {-56.0_deg}; // in - range
            constexpr units::degree_t kMinimumAnglePositive {360_deg + kMinimumAngleNegative}; // in + range
            constexpr units::degree_t kMaximumAngle {kTotalROM + kMinimumAngleNegative};

            constexpr bool kRotationMotorInverted {false};

            constexpr ctre::phoenix6::signals::NeutralModeValue kRotationNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};

        };
    };
};
