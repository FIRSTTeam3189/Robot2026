#include "util/PowerInterpolator.h"

PowerInterpolator::PowerInterpolator(std::vector<units::meter_t> distances, std::vector<double> powers) 
: m_distances(distances), m_powers(powers) {}

double PowerInterpolator::GetPower(frc::Pose2d robotPose, frc::Pose2d targetPose) {

    
    Vector2d robotToTarget(robotPose, targetPose); 
    units::meter_t radius = units::meter_t{robotToTarget.Length()}; 

    frc::SmartDashboard::PutNumber("Power Interpolator/distance", radius.value()); 
    double radiusDouble = radius.value(); 


    int lowerIndex = 0; 
    int upperIndex = m_distances.size() - 1; 

    for (size_t i = 0; i < m_distances.size(); i++) {

        if (m_distances[i] <= radius) {
            lowerIndex = i; 
            continue; 
        }

        if (m_distances[i] > radius) {
            upperIndex = i; 
            break; // After upper index is found, loop is unneccessary 
        }

    }
    
    double r1 = m_distances[lowerIndex].value(); 
    double r2 = m_distances[upperIndex].value(); 

    double p1 = m_powers[lowerIndex];
    double p2 = m_powers[upperIndex]; 

    if (upperIndex == 0 && upperIndex == lowerIndex) {
        r1 = 0;
        p1 = 0; 
    } 
    // If we reach past the end of the vector, return the last power 
    if (upperIndex == m_distances.size() - 1 && upperIndex == lowerIndex) {
        return p2; 
    }

    double slope = (p2 - p1) / (r2 - r1); 
    double deltaRadius = radiusDouble - r1; 
    // mx + b 
    // return slope * deltaRadius + p1; 
    return p1 + slope * deltaRadius;
}