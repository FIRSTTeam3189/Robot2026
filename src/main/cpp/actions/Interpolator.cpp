#include "actions/Interpolator.h"

namespace actions::interpolator {
    frc2::CommandPtr AddInterpolatorPoint(util::interpolator::Data *interpolatorData, subsystems::swerve::Drive *swerveDrive, 
                                          subsystems::shooter::Flywheel *flywheel, subsystems::shooter::Articulation *articulation) {
        return frc2::InstantCommand([interpolatorData, swerveDrive, flywheel, articulation]() { // saves the point 
            frc::Pose2d robotPose = swerveDrive->GetEstimatedPose(); 
            interpolatorData->AddPoint(robotPose, util::align::turret::GetDistanceToTarget(robotPose), 
                                        articulation->GetArticulationAngle(), flywheel->GetFlywheelTargetVelocity().value());
        }, {flywheel, swerveDrive, articulation}).ToPtr();  
    }

    frc2::CommandPtr SaveAllInterpolationPoints(util::interpolator::Data  *interpolatorData) {
        return frc2::InstantCommand([interpolatorData]() {
            interpolatorData->SaveToFile(); 
        }).ToPtr(); 
    }

    frc2::CommandPtr RemoveLastInterpolationPoint(util::interpolator::Data  *interpolatorData) {
        return frc2::InstantCommand([interpolatorData]() {
            interpolatorData->RemoveLastPoint(); 
        }).ToPtr(); 
    }
};

