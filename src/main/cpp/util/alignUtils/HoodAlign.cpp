#include "util/alignUtils/HoodAlign.h"

namespace HoodAlign {

    frc::Pose2d GetHubPose() {
        if (frc::DriverStation::GetAlliance()) {
            return frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue 
                ? AutoAlignConstants::kBlueHubPose : AutoAlignConstants::kRedHubPose; 
        } 
        // bad case 
        return {0.0_m, 0.0_m, frc::Rotation2d(0.0_deg)}; 
    }

    bool BehindHub(frc::Pose2d robotPose) {
        if (!InNeutralZone) {
            return false; 
        }

        frc::Pose2d hubPose = GetHubPose(); 
        double hubY = hubPose.Y().value(); 

        double y = robotPose.Y().value();
        double yMax = hubY + (AutoAlignConstants::kHubWidth.value() / 2);
        double yMin = hubY - (AutoAlignConstants::kHubWidth.value() / 2);

        return y > yMin && y < yMax;
    }

    bool InNeutralZone(frc::Pose2d robotPose) {
        units::meter_t x = robotPose.X(); 
        return x >= AutoAlignConstants::kBlueNeutralZoneThreshold && x <= AutoAlignConstants::kRedNeutralZoneThreshold; 
    }

    units::degree_t CalculateArticulationAngle(frc::Pose2d robotPose) {
        frc::Pose2d hubPose = GetHubPose();

        if (InNeutralZone(robotPose)) {
            if (BehindHub(robotPose)) {
                return AutoAlignConstants::kHoodMinimumAngle;
            }
            return AutoAlignConstants::kHoodMaximumAngle; 
        }

        Vector2d robotToHub(robotPose, hubPose); 
        units::meter_t radius = units::meter_t{robotToHub.Length()}; 

        // units::meter_t y = AutoAlignConstants::kHubHeight - AutoAlignConstants::kHoodHeight;

        double angle = std::atan((AutoAlignConstants::kHoodAndHubHeightDiff / radius).value()) * (180 / PI);

        frc::SmartDashboard::PutNumber("Auto Align/Distance between robot and hub", radius.value()); 
        frc::SmartDashboard::PutNumber("Auto Align/Theta 2", angle);

        units::degree_t rawAngleToHubWithOffset = units::degree_t{(angle + AutoAlignConstants::kHoodDegreeOffset.value())};  // point a bit above hub
        units::degree_t targetHoodAngle = 90.0_deg - rawAngleToHubWithOffset; // convert to angle on other side of hood
        // targetHoodAngle -= AutoAlignConstants::kHoodDefaultAngle; // account for the normal (0 read from encoder) actually at 15 degrees

        frc::SmartDashboard::PutNumber("Auto Align/Hood Angle", targetHoodAngle.value());

        return targetHoodAngle; 
        
    }   


};