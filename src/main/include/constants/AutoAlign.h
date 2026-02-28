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

namespace constants {
    namespace auto_align {
        // These setponts should be manually tuned for shooter angle and power

        constexpr int kNumSetpoints {1};

        // ! TODO
        constexpr frc::Pose2d kBlueKnownSetPoints[kNumSetpoints] = {
            frc::Pose2d (2.182_m, 4.026_m, frc::Rotation2d(0.0_deg)),    
        };

        const std::vector<double> kBlueArticulationAngles = {30.0, 40.0, 45.0};
        const std::vector<double> kBluePowers = {0.5, 0.6, 0.7};

        constexpr frc::Pose2d kRedKnownSetPoints[kNumSetpoints] = {
            frc::Pose2d (5.800_m, 4.025_m, frc::Rotation2d(180.0_deg)),
            // frc::Pose2d (5.145_m, 5.166_m, frc::Rotation2d(240.0_deg)),
            // frc::Pose2d (3.820_m, 5.166_m, frc::Rotation2d(300.0_deg)),
            // frc::Pose2d (3.170_m, 4.025_m, frc::Rotation2d(0.0_deg)),
            // frc::Pose2d (3.830_m, 2.884_m, frc::Rotation2d(60.0_deg)),
            // frc::Pose2d (5.145_m, 2.884_m, frc::Rotation2d(120.0_deg))
        };

        constexpr frc::Pose2d kBlueHubPose {4.624_m, 4.036_m, frc::Rotation2d(0.0_deg)};
        constexpr frc::Pose2d kRedHubPose {11.909_m, 4.036_m, frc::Rotation2d(180.0_deg)};

        constexpr frc::Pose2d kBlueInFrontHubPose {3.624_m, 4.036_m, frc::Rotation2d(0.0_deg)};
        constexpr frc::Pose2d kRedInFrontHubPose {12.909_m, 4.036_m, frc::Rotation2d(180.0_deg)};

        constexpr units::meter_t kBlueNeutralZoneThreshold {4.000_m};
        constexpr units::meter_t kRedNeutralZoneThreshold {12.550_m};

        // ! Not verified numbers 
        constexpr units::meter_t kHubHeight {1.524_m}; 
        constexpr units::meter_t kHoodHeight {0.549_m}; 
        constexpr units::meter_t kHoodAndHubHeightDiff = constants::auto_align::kHubHeight - constants::auto_align::kHoodHeight;

        // 20 deg at tyler's 
        constexpr units::degree_t kHoodDegreeOffset {5.0_deg}; 
        constexpr units::degree_t kHoodDegreeMultiplier {0.34_deg}; 

        constexpr frc::Pose2d kFerryLeftPose {3.07_m, 2.33_m, frc::Rotation2d(0.0_deg)};
        constexpr frc::Pose2d kFerryRightPose {3.07_m, 6.397_m, frc::Rotation2d(0.0_deg)};

        constexpr frc::Pose2d kFerryBluePose {0.466_m, 4.126_m, frc::Rotation2d{0.0_deg}}; 
        constexpr frc::Pose2d kFerryRedPose {16.010_m, 4.126_m, frc::Rotation2d{0.0_deg}}; 

        constexpr units::meter_t kMidpointOnFieldBlue {2.397_m};
        constexpr units::meter_t kMidpointOnFieldRed {14.381_m};

        //4.100->5.057 x , 6.769 y top, 1.106 y bot blue 
        constexpr units::meter_t kBlueTrenchLeftX = 4.100_m; 
        constexpr units::meter_t kBlueTrenchRightX = 5.057_m; 
        //12.440->11.385 x, 
        constexpr units::meter_t kRedTrenchLeftX = 11.385_m; 
        constexpr units::meter_t kRedTrenchRightX = 12.440_m; 

        // Y's for the trench, stay consistent w/alliance
        constexpr units::meter_t kTrenchTop = 6.770_m; 
        constexpr units::meter_t kTrenchBot = 1.105_m; 

        // don't add a k (*-*)
        constexpr units::degree_t TurretAngleLeft {45_deg};
        constexpr units::degree_t TurretAngleRight {135_deg};

        const Vector2d kTurretOffset(0.15, 0.0); 

        constexpr double kMinimumVelocityMultiplier {0.1}; 
        constexpr double kVelocityMultiplier {0.5}; 

        // 0 is 15, and max is 35 
        const std::vector<double> kPowerInterpolatorDistances = {1.0, 1.5, 2.0, 2.5, 3.0}; 
        const std::vector<double> kPowerInterpolatorPowers = {0.60, 0.66, 0.73, 0.84, 0.95}; 

        //! need to be configured 
        constexpr units::degree_t kHoodDefaultAngle{15_deg};

        // 15 is like suoper high arc and 50 is lowwww
        constexpr units::degree_t kHoodMinimumAngle{15_deg};
        constexpr units::degree_t kHoodMaximumAngle{50_deg};

        constexpr units::meter_t kBlueTrenchRight {5.200_m};
        constexpr units::meter_t kRedTrenchLeft {11.350_m};


        constexpr units::meter_t kHubWidth {1.2_m}; 
        
    };
};
