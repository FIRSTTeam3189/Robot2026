#include "util/PowerInterpolator.h"

PowerInterpolator::PowerInterpolator(std::vector<double> angles, std::vector<double> powers) 
: m_angles(angles), m_powers(powers) {}

double PowerInterpolator::GetPower(double hoodAngle) {



    frc::SmartDashboard::PutNumber("Power Interpolator/angle", hoodAngle);

    int size = m_angles.size(); 
    int lowerIndex = 0; 
    int upperIndex = size - 1; 

    for (size_t i = 0; i < size; i++) {

        if (m_angles[i] <= hoodAngle) {
            lowerIndex = i; 
            continue; 
        }

        if (m_angles[i] > hoodAngle) {
            upperIndex = i; 
            break; // After upper index is found, loop is unneccessary 
        }

    }
    
    double r1 = m_angles[lowerIndex]; 
    double r2 = m_angles[upperIndex]; 

    double p1 = m_powers[lowerIndex];
    double p2 = m_powers[upperIndex]; 

    if (upperIndex == 0 && upperIndex == lowerIndex) {
        r1 = 0;
        p1 = 0; 
    } 
    // If we reach past the end of the vector, return the last power 
    if (upperIndex == size - 1 && upperIndex == lowerIndex) {
        return p2; 
    }

    double slope = (p2 - p1) / (r2 - r1); 
    double deltaAngle = hoodAngle - r1; 
    // mx + b 
    // return slope * deltaAngle + p1; 
    return p1 + slope * deltaAngle;
}