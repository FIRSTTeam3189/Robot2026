#pragma once
#include <frc/geometry/Pose2d.h>

namespace AutoAlignConstants{

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

    constexpr frc::Pose2d HubAngle {6.7_m, 6.7_m, frc::Rotation2d(0.0_deg)};
    constexpr frc::Pose2d FerryAngle {6.9_m, 6.9_m, frc::Rotation2d(0.0_deg)};
}