#include "util/interpolator/Linear.h"


namespace util::interpolator {

    Linear::Linear(std::vector<double> values1, std::vector<double> values2) 
    : m_values1(values1), m_values2(values2) {}
    
    double Linear::Predict(double value1) {
        int size = m_values1.size(); 
        int lowerIndex = 0; 
        int upperIndex = size - 1; 
    
        for (size_t i = 0; i < size; i++) {
    
            if (m_values1[i] <= value1) {
                lowerIndex = i; 
                continue; 
            }
    
            if (m_values1[i] > value1) {
                upperIndex = i; 
                break; // After upper index is found, loop is unneccessary 
            }
    
        }
        
        double r1 = m_values1[lowerIndex]; 
        double r2 = m_values1[upperIndex]; 
    
        double p1 = m_values2[lowerIndex];
        double p2 = m_values2[upperIndex]; 
    
        if (upperIndex == 0 && upperIndex == lowerIndex) {
            r1 = 0;
            p1 = 0; 
        } 
    
        // If we reach past the end of the vector, return the last power 
        if (upperIndex == size - 1 && upperIndex == lowerIndex) {
            return p2; 
        }
    
        double slope = (p2 - p1) / (r2 - r1); 
        double delta = value1 - r1; 
        // mx + b 
        // return slope * delta + p1; 
        return p1 + slope * delta;
    }
}
