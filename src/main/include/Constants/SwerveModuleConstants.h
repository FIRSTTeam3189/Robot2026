#pragma once

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/time.h>
#include <units/frequency.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#define PI 3.14159265358979323846

                                                                                                           
#define SWERVEMODULECONSTANTS2025

#ifdef SWERVEMODULECONSTANTS2025
// 2025
    inline namespace SwerveModuleConstants {
        // Sensor IDs for motors + encoders - labeled on robot
        constexpr int kFrontLeftDriveID {1};
        constexpr int kFrontLeftAngleID {2};

        constexpr int kFrontRightDriveID {3};
        constexpr int kFrontRightAngleID {4};

        constexpr int kBackLeftDriveID {5};
        constexpr int kBackLeftAngleID {6};

        constexpr int kBackRightDriveID {7};
        constexpr int kBackRightAngleID {8};

        constexpr int kFrontLeftCANcoderID {9};
        constexpr int kFrontRightCANcoderID {10};
        constexpr int kBackLeftCANcoderID {11};
        constexpr int kBackRightCANcoderID {12};

        // Swerve angle offsets -- difference between actual degrees heading and absolute degree values
        constexpr units::turn_t kFrontLeftOffset {0.449951};
        constexpr units::turn_t kFrontRightOffset {0.490967};
        constexpr units::turn_t kBackLeftOffset {0.244385}; 
        constexpr units::turn_t kBackRightOffset {0.471436};

        // Motor + sensor inversions
        constexpr bool kDriveMotorInverted = true;
        constexpr bool kAngleMotorInverted = true;
        constexpr bool kCANcoderInverted = false;

        constexpr units::volt_t kMaxAngleVoltage {12.0};
        // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
        constexpr units::ampere_t kAngleContinuousCurrentLimit = units::ampere_t{30.0};
        constexpr units::ampere_t kAnglePeakCurrentLimit = units::ampere_t{45.0};
        constexpr units::second_t kAnglePeakCurrentDuration = 0.5_s; // seconds
        constexpr bool kAngleEnableCurrentLimit = true;
        
        constexpr units::ampere_t kDriveContinuousCurrentLimit = units::ampere_t{35.0};
        constexpr units::ampere_t kDrivePeakCurrentLimit = units::ampere_t{65.0};
        constexpr units::second_t kDrivePeakCurrentDuration = 0.5_s;
        constexpr bool kDriveEnableCurrentLimit = true; //meow -

        // Encoder sensor range
        // constexpr auto kCANcoderSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
        constexpr units::turn_t kCANcoderDiscontinuityPoint = units::turn_t{0.5}; // same as +- half from before

        // Motor neutral modes -- what they do when no power is applied
        constexpr ctre::phoenix6::signals::NeutralModeValue kDriveNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        constexpr ctre::phoenix6::signals::NeutralModeValue kAngleNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr double kDriveNeutralDeadband = 0.02;

        // TalonFX Remote Sensor Type Settings
        constexpr int kRotorSensor{0};
        constexpr int kRemoteCANcoder{1};
        constexpr int kRemotePigeon2_Yaw{2};
        constexpr int kRemotePigeon2_Pitch{3};
        constexpr int kRemotePigeon2_Roll{4};
        constexpr int kFusedCANcoder{5};
        constexpr int kSyncCANcoder{6};

        // Even though probably not using FF, keep constants in case
        constexpr double kPDrive {2.5};
        constexpr double kIDrive {0.0};
        constexpr double kDDrive {0.0};
        constexpr double kVDrive {0.0};
        constexpr double kSDrive {0.0};

        constexpr double kPAngle {24.0};
        constexpr double kIAngle {0.0};
        constexpr double kDAngle {0.0};
        constexpr double kVAngle {0.0};
        constexpr double kSAngle {0.0};

        constexpr units::meters_per_second_t kMaxSpeed {5.75_mps};
        constexpr double kModuleSpeedMultiplier {1.311};
        constexpr double kDEGToRAD {57.2957795131};
        constexpr double kWheelRadiusInches {2.0};
        constexpr double kWheelRadiusMeters {0.0508};
        constexpr double kWheelCircumferenceMeters {2.0 * PI * kWheelRadiusMeters};
        constexpr double kDriveGearRatio {6.11};
        constexpr double kAngleGearRatio {13.3714};
        constexpr double kRotationsPerMeter {1.0 / kWheelCircumferenceMeters};
        constexpr int kFalconEncoderTicksPerRevolution {2048};
        constexpr int kCANcoderTicksPerRevolution {4096};
        constexpr double kWheelCOF {1.200}; // estimated
    }

// 2026
#else
    inline namespace SwerveModuleConstants{
        // Sensor IDs for motors + encoders - labeled on robot
        constexpr int kFrontLeftDriveID {1};
        constexpr int kFrontLeftAngleID {2};

        constexpr int kFrontRightDriveID {3};
        constexpr int kFrontRightAngleID {4};

        constexpr int kBackLeftDriveID {5};
        constexpr int kBackLeftAngleID {6};

        constexpr int kBackRightDriveID {7};
        constexpr int kBackRightAngleID {8};

        constexpr int kFrontLeftCANcoderID {9};
        constexpr int kFrontRightCANcoderID {10};
        constexpr int kBackLeftCANcoderID {11};
        constexpr int kBackRightCANcoderID {12};

        // Swerve angle offsets -- difference between actual degrees heading and absolute degree values
        constexpr units::turn_t kFrontLeftOffset {-0.267090}; // TBD
        constexpr units::turn_t kFrontRightOffset {-0.405518};// TBD+
        constexpr units::turn_t kBackLeftOffset {0.265625}; // TBD
        constexpr units::turn_t kBackRightOffset {-0.114258};// TBD

        // Motor + sensor inversions
        constexpr bool kDriveMotorInverted = true;
        constexpr bool kAngleMotorInverted = false;
        constexpr bool kCANcoderInverted = true;

        constexpr units::volt_t kMaxAngleVoltage {12.0};
        // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
        constexpr units::ampere_t kAngleContinuousCurrentLimit = units::ampere_t{30.0};
        constexpr units::ampere_t kAnglePeakCurrentLimit = units::ampere_t{45.0};
        constexpr units::second_t kAnglePeakCurrentDuration = 0.5_s; // seconds
        constexpr bool kAngleEnableCurrentLimit = true;
        
        constexpr units::ampere_t kDriveContinuousCurrentLimit = units::ampere_t{35.0};
        constexpr units::ampere_t kDrivePeakCurrentLimit = units::ampere_t{65.0};
        constexpr units::second_t kDrivePeakCurrentDuration = 0.5_s;
        constexpr bool kDriveEnableCurrentLimit = true; //meow -

        // Encoder sensor range
        // constexpr auto kCANcoderSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
        constexpr units::turn_t kCANcoderDiscontinuityPoint = units::turn_t{0.5}; // same as +- half from before

        // Motor neutral modes -- what they do when no power is applied
        constexpr ctre::phoenix6::signals::NeutralModeValue kDriveNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        constexpr ctre::phoenix6::signals::NeutralModeValue kAngleNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr double kDriveNeutralDeadband = 0.02;

        // TalonFX Remote Sensor Type Settings
        constexpr int kRotorSensor{0};
        constexpr int kRemoteCANcoder{1};
        constexpr int kRemotePigeon2_Yaw{2};
        constexpr int kRemotePigeon2_Pitch{3};
        constexpr int kRemotePigeon2_Roll{4};
        constexpr int kFusedCANcoder{5};
        constexpr int kSyncCANcoder{6};

        // Even though probably not using FF, keep constants in case
        constexpr double kPDrive {1.5};
        constexpr double kIDrive {0.0};
        constexpr double kDDrive {0.0};
        constexpr double kVDrive {0.0};
        constexpr double kSDrive {0.0};

        
        constexpr double kPAngle {17.5};
        constexpr double kIAngle {0.0};
        constexpr double kDAngle {0.0};
        constexpr double kVAngle {0.0};
        constexpr double kSAngle {0.0};

        constexpr units::meters_per_second_t kMaxSpeed {5.75_mps};
        constexpr double kModuleSpeedMultiplier {1.311};
        constexpr double kDEGToRAD {57.2957795131};

        constexpr double kWheelRadiusInches {1.75};
        constexpr double kWheelRadiusMeters {0.0444};
        constexpr double kWheelCircumferenceMeters {2.0 * PI * kWheelRadiusMeters};

        constexpr double kDriveGearRatio {4.42};
        constexpr double kAngleGearRatio {25.91};

        constexpr double kRotationsPerMeter {1.0 / kWheelCircumferenceMeters};
    }
#endif
