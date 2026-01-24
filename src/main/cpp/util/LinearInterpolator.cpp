// #include "util/LinearInterpolator.h"


// template<typename P, typename K>
// LinearInterpolator<P, K>::LinearInterpolator(const std::vector<P> &xValues, const std::vector<K> &yValues,
//                                              std::function<double(P)> pToDouble, std::function<double(K)> kToDouble) 
// : m_xValues(xValues), m_yValues(yValues), m_PToDouble(pToDouble), m_KToDouble(kToDouble) {}

// template<typename P, typename K>
// K LinearInterpolator<P, K>::Predict(P x) {
//     double radiusDouble = radius.value(); 


//     int lowerIndex = 0; 
//     int upperIndex = m_distances.size() - 1; 

//     for (size_t i = 0; i < m_distances.size(); i++) {

//         if (m_distances[i] <= radius) {
//             lowerIndex = i; 
//             continue; 
//         }

//         if (m_distances[i] > radius) {
//             upperIndex = i; 
//             break; // After upper index is found, loop is unneccessary 
//         }

//     }
    
//     // Convert P to double 
//     double r1 = m_PToDouble(m_distances[lowerIndex]); 
//     double r2 = m_PToDouble(m_distances[upperInd


//     // Convert K to double 
//     double p1 = m_KToDouble(m_powers[lowerIndex]);
//     double p2 = m_KToDouble(m_powers[upperIndex]); 

//     if (upperIndex == 0 && upperIndex == lowerIndex) {
//         r1 = 0;
//         p1 = 0; 
//     } 
//     // If we reach past the end of the vector, return the last power 
//     if (upperIndex == m_distances.size() - 1 && upperIndex == lowerIndex) {
//         return p2; 
//     }

//     double slope = (p2 - p1) / (r2 - r1); 
//     double deltaRadius = radiusDouble - r1; 
//     // mx + b 
//     // return slope * deltaRadius + p1; 
//     return p1 + slope * deltaRadius;
// }
