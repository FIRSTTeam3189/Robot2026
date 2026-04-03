#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <iostream>

namespace constants {
    namespace vision {
        constexpr wpi::array<double, 3> kEncoderTrustCoefficients {0.1, 0.1, 0.1};
        // !!! RAHHHH 
        // TODO
        constexpr wpi::array<double, 3> kMainVisionTrustCoefficients {0.2, 0.2, 10.0};
        constexpr double kMainVisionStdDevPerMeter {0.3};
        constexpr wpi::array<double, 3> kPhotonVisionTrustCoefficients {0.5, 0.5, 10.0};
        constexpr double kPhotonVisionStdDevPerMeter {0.25};  // TODOOOOOOO

        
        // 41 degrees measure from plate to camera
        // 2.4 in radius 
        // 0.04 m offset from the bottom side of triangle formed with camera and mounting plate
        // 0.0254 m is the center point where the camera actually lies approx with measuring tape 
        const std::string kPhotonTurretFrontCamera {"Front Turret Camera"};
        // constexpr units::meter_t kPhotonTurretFrontCameraXOffset {-0.3048508_m}; 
        // constexpr units::meter_t kPhotonTurretFrontCameraYOffset {-0.1843532_m}; without offset 
        constexpr units::meter_t kPhotonTurretFrontCameraXOffset {-0.3_m}; // with 0.0254 offset 
        constexpr units::meter_t kPhotonTurretFrontCameraYOffset {-0.14_m}; 
        constexpr units::meter_t kPhotonTurretFrontCameraZOffset {0.0_m};
        constexpr units::meter_t kPhotonTurretFrontCameraHeight {0.0_m};
        constexpr units::degree_t kPhotonTurretFrontCameraPitchOffset {0.0_deg}; 
        constexpr units::degree_t kPhotonTurretFrontCameraYawOffset {0.0_deg}; 

        const std::string kPhotonTurretRightCamera {"Right Cam"};
        constexpr units::meter_t kPhotonTurretRightCameraXOffset {-0.29_m}; 
        constexpr units::meter_t kPhotonTurretRightCameraYOffset {-0.21_m}; 
        constexpr units::meter_t kPhotonTurretRightCameraZOffset {0.0_m};
        constexpr units::meter_t kPhotonTurretRightCameraHeight {0.0_m};
        constexpr units::degree_t kPhotonTurretRightCameraPitchOffset {0.0_deg};
        constexpr units::degree_t kPhotonTurretRightCameraYawOffset {-90.0_deg};

        const std::string kPhotonTurretLeftCamera {"Left Cam"};
        constexpr units::meter_t kPhotonTurretLeftCameraXOffset {-0.29_m}; 
        constexpr units::meter_t kPhotonTurretLeftCameraYOffset {-0.21_m}; 
        constexpr units::meter_t kPhotonTurretLeftCameraZOffset {0.0_m};
        constexpr units::meter_t kPhotonTurretLeftCameraHeight {0.0_m};
        constexpr units::degree_t kPhotonTurretLeftCameraPitchOffset {0.0_deg}; 
        constexpr units::degree_t kPhotonTurretLeftCameraYawOffset {90.0_deg}; 

        constexpr int kBaudRate {115200};
        constexpr int kBufferSize {1024};

        constexpr double kMaxAmbiguity {0.2}; // [0, 1] where 1 is completely ambiguous
        constexpr bool kShouldUseVision {true};
        constexpr double kUseVisionMeasurementThreshold {100.0}; // Meters
        constexpr units::meter_t kMaxDistance {100.0_m};
        
        // Vision Sync bytes, in char format
        constexpr std::array kSyncBytes {'\x1a', '\xcf', '\xfc', '\x1d'};
    };
};