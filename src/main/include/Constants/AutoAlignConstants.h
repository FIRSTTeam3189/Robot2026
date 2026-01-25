#pragma once
#include <frc/geometry/Pose2d.h>
#include "util/Vector2d.h" 
#include <units/time.h>
#include <units/frequency.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <vector>

namespace AutoAlignConstants {

    // These setponts should be manually tuned for shooter angle and power
    constexpr int kNumSetpoints {6};


    // ! TODO
    constexpr frc::Pose2d kBlueKnownSetPoints[kNumSetpoints] = {
        frc::Pose2d (5.800_m, 4.025_m, frc::Rotation2d(180.0_deg)),
        frc::Pose2d (5.145_m, 5.166_m, frc::Rotation2d(240.0_deg)),
        frc::Pose2d (3.820_m, 5.166_m, frc::Rotation2d(300.0_deg)),
        frc::Pose2d (3.170_m, 4.025_m, frc::Rotation2d(0.0_deg)),
        frc::Pose2d (3.830_m, 2.884_m, frc::Rotation2d(60.0_deg)),
        frc::Pose2d (5.145_m, 2.884_m, frc::Rotation2d(120.0_deg))
    };

    constexpr frc::Pose2d kRedKnownSetPoints[kNumSetpoints] = {
        frc::Pose2d (5.800_m, 4.025_m, frc::Rotation2d(180.0_deg)),
        frc::Pose2d (5.145_m, 5.166_m, frc::Rotation2d(240.0_deg)),
        frc::Pose2d (3.820_m, 5.166_m, frc::Rotation2d(300.0_deg)),
        frc::Pose2d (3.170_m, 4.025_m, frc::Rotation2d(0.0_deg)),
        frc::Pose2d (3.830_m, 2.884_m, frc::Rotation2d(60.0_deg)),
        frc::Pose2d (5.145_m, 2.884_m, frc::Rotation2d(120.0_deg))
    };

    constexpr frc::Pose2d kBlueHubPose {4.624_m, 4.036_m, frc::Rotation2d(0.0_deg)};
    constexpr frc::Pose2d kRedHubPose {11.909_m, 4.036_m, frc::Rotation2d(0.0_deg)};

    // ! Not verified numbers 
    constexpr units::meter_t kHubHeight {1.524_m}; 
    constexpr units::meter_t kHoodHeight {0.6096_m}; 
    constexpr units::meter_t ky = AutoAlignConstants::kHubHeight - AutoAlignConstants::kHoodHeight;

    constexpr units::degree_t kHoodDegreeOffset {2.0_deg}; 

    constexpr frc::Pose2d kFerryLeftPose {3.07_m, 2.33_m, frc::Rotation2d(0.0_deg)};
    constexpr frc::Pose2d kFerryRightPose {3.07_m, 6.397_m, frc::Rotation2d(0.0_deg)};

    constexpr units::meter_t kMidpointOnFieldBlue {2.397_m};
    constexpr units::meter_t kMidpointOnFieldRed {14.381_m};

    constexpr units::degree_t FerryAngleLeft {45_deg};
    constexpr units::degree_t FerryAngleRight {135_deg};

    const Vector2d kTurretOffset(0.11, 0.0); 

    constexpr double kMinimumVelocityMultiplier {0.1}; 
    constexpr double kVelocityMultiplier {0.5}; 

    // 0 is 15, and max is 35 
    const std::vector<double> kPowerInterpolatorAngles = {10, 15, 20, 25, 30, 35}; 
    const std::vector<double> kPowerInterpolatorPowers = {0.25, 0.34, 0.51, 0.62, 0.8}; 

    constexpr double kHoodDefaultAngle{15};

    
}