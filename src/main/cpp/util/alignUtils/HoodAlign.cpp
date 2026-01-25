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

    double CalculateArticulationAngle(frc::Pose2d robotPose) {
        frc::Pose2d hubPose = GetHubPose(); 

        Vector2d robotToHub(robotPose, hubPose); 
        units::meter_t radius = units::meter_t{robotToHub.Length()}; 

        units::meter_t y = AutoAlignConstants::kHubHeight - AutoAlignConstants::kHoodHeight;

        double angle = std::atan((y / radius).value());

        frc::SmartDashboard::PutNumber("Distance between robot and hub", radius.value()); 

        return angle + AutoAlignConstants::kHoodDegreeOffset.value(); 
    }   


};
