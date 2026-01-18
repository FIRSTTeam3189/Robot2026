#include "util/PowerInterpolater.h"


PowerInterpolater::PowerInterpolater(std::vector<units::meter_t> distances, std::vector<double> powers) 
: m_distances(distances), m_powers(powers) {}

double PowerInterpolater::GetPower(units::meter_t radius) {
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

    double slope = (p2 - p1) / (r2 - r1); 
    // mx + b 
    return slope * radiusDouble + p1; 
}