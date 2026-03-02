#include "util/align/Hood.h"


namespace util::align::hood {

    frc::Pose2d GetHubPose() {
        if (frc::DriverStation::GetAlliance()) {
            return frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue 
                ? constants::auto_align::kBlueHubPose : constants::auto_align::kRedHubPose; 
        } 
        // bad case 
        return {0.0_m, 0.0_m, frc::Rotation2d(0.0_deg)}; 
    }

    bool InNeutralZone(frc::Pose2d robotPose) {
        units::meter_t x = robotPose.X(); 
        return x >= constants::auto_align::kBlueNeutralZoneThreshold && x <= constants::auto_align::kRedNeutralZoneThreshold; 
    }


    bool BehindHub(frc::Pose2d robotPose) {
        if (!InNeutralZone) {
            return false; 
        }

        frc::Pose2d hubPose = GetHubPose(); 
        double hubY = hubPose.Y().value(); 

        double y = robotPose.Y().value();
        double yMax = hubY + (constants::auto_align::kHubWidth.value() / 2);
        double yMin = hubY - (constants::auto_align::kHubWidth.value() / 2);

        return y > yMin && y < yMax;
    }

    bool InTrench(frc::Pose2d robotPose) {
        units::meter_t x = robotPose.X();
        bool inBlueTrench = x >= constants::auto_align::kBlueNeutralZoneThreshold && x <= constants::auto_align::kBlueTrenchRight;
        bool inRedTrench = x <= constants::auto_align::kRedNeutralZoneThreshold && x >= constants::auto_align::kRedTrenchLeft;

        return inBlueTrench || inRedTrench;
    }

    units::degree_t CalculateArticulationAngle(frc::Pose2d robotPose) {
        frc::Pose2d hubPose = GetHubPose();

        if (InTrench(robotPose)) {
            return constants::auto_align::kHoodMinimumAngle; 
        }

        if (InNeutralZone(robotPose)) {
            if (BehindHub(robotPose)) {
                return constants::auto_align::kHoodMinimumAngle;
            }
            return constants::auto_align::kHoodMaximumAngle; 
        }

        Vector2d robotToHub(robotPose, hubPose); 
        units::meter_t radius = units::meter_t{robotToHub.Length()}; 

        double angle = std::atan((constants::auto_align::kHoodAndHubHeightDiff / radius).value()) * (180 / PI);

        frc::SmartDashboard::PutNumber("Auto Align/Distance between robot and hub", radius.value()); 
        frc::SmartDashboard::PutNumber("Auto Align/Angle from shooter to top of hub", angle);

        // units::degree_t rawAngleToHubScaled = units::degree_t{angle * constants::auto_align::kHoodDegreeMultiplier.value()}; // make bias towards hood being lower  
        units::degree_t rawAngleToHubWithOffset = units::degree_t{angle + constants::auto_align::kHoodDegreeOffset.value()};  // point a bit above hub
        units::degree_t targetHoodAngle = 90.0_deg - rawAngleToHubWithOffset; // convert to angle on other side of hood
        units::degree_t targetHoodAngleScaled = units::degree_t{targetHoodAngle.value() * constants::auto_align::kHoodDegreeMultiplier.value()}; // make bias towards hood being lower  

        // targetHoodAngle -= constants::auto_align::kHoodDefaultAngle; // account for the normal (0 read from encoder) actually at 15 degrees
        
        frc::SmartDashboard::PutNumber("Auto Align/Target Hood Angle with offset", targetHoodAngle.value());
        frc::SmartDashboard::PutNumber("Auto Align/Target Hood Scaled Angle", targetHoodAngleScaled.value()); 
        frc::SmartDashboard::PutNumber("Auto Align/Target Hood Angle without offset", (90.0 - angle));
        
        //clamp it 
        targetHoodAngleScaled+=2.5_deg; 
        if (targetHoodAngleScaled > 32.5_deg) {
            targetHoodAngleScaled = 32.5_deg;
        }

        return targetHoodAngleScaled; 
        
    }   


};