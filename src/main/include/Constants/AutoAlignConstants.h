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

    constexpr frc::Pose2d kHubPose {4.624_m, 4.023_m, frc::Rotation2d(0.0_deg)};
    constexpr frc::Pose2d kFerryLeftPose {3.07_m, 2.33_m, frc::Rotation2d(0.0_deg)};
    constexpr frc::Pose2d kFerryRightPose {3.07_m, 6.397_m, frc::Rotation2d(0.0_deg)};

    constexpr units::degree_t FerryAngleLeft {45_deg};
    constexpr units::degree_t FerryAngleRight {135_deg};

    const Vector2d kTurretOffset(0.0, 0.0); 

}