#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <iostream>

namespace VisionConstants {
    constexpr wpi::array<double, 3> kEncoderTrustCoefficients {0.1, 0.1, 0.1};
    // !!! RAHHHH 
    // TODO
    constexpr wpi::array<double, 3> kMainVisionTrustCoefficients {0.2, 0.2, 10.0};
    constexpr double kMainVisionStdDevPerMeter {0.3};
    constexpr wpi::array<double, 3> kPhotonVisionTrustCoefficients {0.5, 0.5, 10.0};
    constexpr double kPhotonVisionStdDevPerMeter {0.25};  // TODOOOOOOO

    // !RAHHH Constants are wrong (prolly)!!!!
    // These should be position not as a transform (positive)
    constexpr units::meter_t kMainCameraXOffset {0.29_m}; 
    constexpr units::meter_t kMainCameraYOffset {0.0_m}; 
    constexpr units::meter_t kMainCameraZOffset {0.0_m};
    constexpr units::degree_t kMainCameraPitchOffset {10.0_deg}; 
    constexpr units::degree_t kMainCameraYawOffset {0.0_deg};
    constexpr units::meter_t kMaxDistance {100.0_m};
    constexpr double kMaxAmbiguity {0.2}; // [0, 1] where 1 is completely ambiguous
    constexpr bool kShouldUseVision {true};
    constexpr double kUseVisionMeasurementThreshold {100.0}; // Meters



    //! START VERSION THAT WE DID JANKILY //////

    // const std::string kPhotonIntakeCameraName {"Camera_Reef_Center"};
    // constexpr units::meter_t kPhotonIntakeCameraXOffset {-0.04179_m}; 
    // constexpr units::meter_t kPhotonIntakeCameraYOffset {0.2032_m}; 
    // constexpr units::meter_t kPhotonIntakeCameraZOffset {0.0_m};
    // constexpr units::degree_t kPhotonIntakeCameraPitchOffset {0.0_deg}; 
    // constexpr units::degree_t kPhotonIntakeCameraYawOffset {180.0_deg}; 


    // const std::string kPhotonReefCameraName {"Camera_Intake"};
    // constexpr units::meter_t kPhotonReefCameraXOffset {-0.18519_m}; 
    // constexpr units::meter_t kPhotonReefCameraYOffset {0.5032_m}; 
    // constexpr units::meter_t kPhotonReefCameraZOffset {0.0_m}; // don't care
    // constexpr units::degree_t kPhotonReefCameraPitchOffset {0.0_deg};  // don't care
    // constexpr units::degree_t kPhotonReefCameraYawOffset {0.0_deg};

    // const std::string kPhotonTurretCameraName {"Camera_Reef"};
    // constexpr units::meter_t kPhotonTurretCameraXOffset {-0.29_m}; 
    // constexpr units::meter_t kPhotonTurretCameraYOffset {0.0_m}; 
    // constexpr units::meter_t kPhotonTurretCameraZOffset {0.0_m};
    // constexpr units::degree_t kPhotonTurretCameraPitchOffset {0.0_deg}; 
    // constexpr units::degree_t kPhotonTurretCameraYawOffset {0.0_deg}; 
    //! END VERSION THAT WE DID JANKILY //////

    

    const std::string kPhotonTurretFrontCamera {"Front Turret Camera"};
    constexpr units::meter_t kPhotonTurretFrontCameraXOffset {-0.3048_m}; 
    constexpr units::meter_t kPhotonTurretFrontCameraYOffset {-0.1778_m}; 
    constexpr units::meter_t kPhotonTurretFrontCameraZOffset {0.0_m};
    constexpr units::meter_t kPhotonTurretFrontCameraHeight {0.135_m};
    constexpr units::degree_t kPhotonTurretFrontCameraPitchOffset {33.0_deg}; 
    constexpr units::degree_t kPhotonTurretFrontCameraYawOffset {0.0_deg}; 

    const std::string kPhotonCamera2 {"Vision 2"};
    constexpr units::meter_t kPhotonReefCameraXOffset {-0.18519_m}; 
    constexpr units::meter_t kPhotonReefCameraYOffset {0.2032_m}; 
    constexpr units::meter_t kPhotonReefCameraZOffset {0.0_m}; // don't care
    constexpr units::degree_t kPhotonReefCameraPitchOffset {0.0_deg};  // don't care
    constexpr units::degree_t kPhotonReefCameraYawOffset {0.0_deg};

    const std::string kPhotonCamera3 {"Turret Camera"};
    constexpr units::meter_t kPhotonTurretCameraXOffset {0.23_m}; 
    constexpr units::meter_t kPhotonTurretCameraYOffset {0.046_m}; 
    constexpr units::meter_t kPhotonTurretCameraZOffset {0.0_m};
    constexpr units::degree_t kPhotonTurretCameraPitchOffset {35.2_deg}; 
    constexpr units::degree_t kPhotonTurretCameraYawOffset {0.0_deg}; 

    constexpr int kBaudRate {115200};
    constexpr int kBufferSize {1024};

    constexpr units::meter_t kRobotRelativeBranchOffsetX {0.546_m}; 
    constexpr units::meter_t kRobotRelativeBranchOffsetY {0.1651_m}; // Distance between center of each reef face and center of pipe (or pipe to pipe / 2)
    constexpr units::meter_t kOffsetMagnitude {0.5704156_m}; // Just distance formula with two above

    // Vision Sync bytes, in char format
    constexpr std::array kSyncBytes {'\x1a', '\xcf', '\xfc', '\x1d'};
}