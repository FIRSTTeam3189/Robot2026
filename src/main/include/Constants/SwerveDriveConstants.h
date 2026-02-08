#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/frequency.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#define PI 3.14159265358979323846

enum class ActiveDriveController { OfficialDriver, TestControls };

namespace SwerveDriveConstants {
    // SET ACTIVE DRIVER HERE
    constexpr ActiveDriveController kActiveController {ActiveDriveController::OfficialDriver};

    constexpr int kGyroID {13};
    constexpr double kRadiansToDegreesMultiplier {180.0 / PI};

    // Coordinate plane distance in meters to each swerve drive
    // This has x-positive as forward, y-positive as left
    // constexpr units::meter_t kXDistanceFromCenter {0.27125_m};
    // constexpr units::meter_t kYDistanceFromCenter {0.27125_m};
    // constexpr units::meter_t kWheelbase {0.27125_m * 2.0};
    // constexpr units::meter_t kTrackwidth {0.27125_m * 2.0};

    // constexpr units::meter_t kXDistanceFromCenter {0.307975_m};
    // constexpr units::meter_t kYDistanceFromCenter {0.307975_m};
    // constexpr units::meter_t kWheelbase {0.307975_m * 2.0};
    // constexpr units::meter_t kTrackwidth {0.307975_m * 2.0};

    constexpr units::meter_t kXDistanceFromCenter {0.3048};
    constexpr units::meter_t kYDistanceFromCenter {0.3048};
    constexpr units::meter_t kWheelbase {0.3048 * 2.0};
    constexpr units::meter_t kTrackwidth {0.3048 * 2.0};

    constexpr units::degree_t kGyroMountPoseYaw {0.0};

    constexpr double kMidFieldY = 4.026;  // Meters
   

    // constexpr frc::Pose2d kBlueResetPose {1.4616_m, 4.026_m, frc::Rotation2d{0.0_deg}};
    // constexpr frc::Pose2d kBlueResetPose {2.366_m, 4.151_m, frc::Rotation2d{0.0_deg}};
    constexpr frc::Pose2d kBlueResetPose {2.9_m, 4.026_m, frc::Rotation2d{180.0_deg}};

    // constexpr frc::Pose2d kBlueResetPose {7.572_m, 4.026_m, frc::Rotation2d{0.0_deg}};
    constexpr frc::Pose2d kRedResetPose {4.904_m, 6.1_m, frc::Rotation2d{0.0_deg}};

    constexpr units::degree_t kClimbAlignRotBlue {90_deg};
    constexpr units::degree_t kClimbAlignRotRed {-90_deg}; 

    constexpr units::hertz_t kRefreshRate = units::hertz_t {100};

    static frc::SwerveDriveKinematics<4> kKinematics {
        frc::Translation2d{+SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{+SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{-SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{-SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter}
    };
    constexpr units::meters_per_second_t kMaxSpeed {5.0_mps};
    constexpr units::radians_per_second_t kMaxAngularVelocity {3.5 * PI};
    constexpr double kActualMaxRotationalVelocity {4.0};
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration {2.0 * PI};
    // SysID robot characterization values -- **varies by robot**
    // constexpr double kSRot {0.125}; // 0.408_V
    // constexpr auto kv {3.206 * 1_V * 1_s / 1_m};
    // constexpr auto ka {3.409 * 1_V * 1_s * 1_s / 1_m};

    // These are for robot rotation, not wheel rotation
    constexpr double kSRot {0.125}; // 0.408_V

    // These are for robot rotation, not wheel rotation
    constexpr double kPRot {0.0075};
    constexpr double kIRot {0.0}; // constexpr double kIRot {0.00025};
    constexpr double kDRot {0.0};

    
    // !! OLD DON'T WORRY ABOUT IT
    // constexpr frc::Pose2d kPathEndTolerance {0.05_m, 0.05_m, 2.5_deg};  // For all paths for now in teleop

    constexpr units::meter_t kTranslationTolerance {0.05_m};
    constexpr auto kRotationalTolerance {2.5_deg};

    

    constexpr double kSlowModeDriveMultiplier {0.35};
    constexpr double kSwerveRotationTolerance {1.0};

    constexpr double kSlowModeSwerveDriveMultiplier {0.142857};
    constexpr double kSlowModeSwerveDriveRotMultiplier {0.25};


    

    
}