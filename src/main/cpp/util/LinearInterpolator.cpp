// #include "util/LinearInterpolator.h"


// template<typename P, typename K>
// LinearInterpolator<P, K>::LinearInterpolator(const std::vector<P> &xValues, const std::vector<K> &yValues,
//                                              std::function<double(P)> pToDouble, std::function<double(K)> kToDouble)
// : m_xValues(xValues), m_yValues(yValues), m_PToDouble(std::move(pToDouble)), m_KToDouble(std::move(kToDouble)) {}

// template<typename P, typename K>
// K LinearInterpolator<P, K>::Predict(P x) {
//     double radius = m_PToDouble(x); 


//     int lowerIndex = 0; 
//     int size = m_xValues.size(); 
//     int upperIndex = size - 1; 

//     for (size_t i = 0; i < size; i++) {

//         if (m_xValues[i] <= radius) {
//             lowerIndex = i; 
//             continue; 
//         }

//         if (m_xValues[i] > radius) {
//             upperIndex = i; 
//             break; // After upper index is found, loop is unneccessary 
//         }

//     }
    
//     // Convert P to double 
//     double r1 = m_PToDouble(m_xValues[lowerIndex]); 
//     double r2 = m_PToDouble(m_xValues[upperIndex]); 

//     // Convert K to double 
//     double p1 = m_KToDouble(m_yValues[lowerIndex]);
//     double p2 = m_KToDouble(m_yValues[upperIndex]); 

//     // Start from the origin 
//     if (upperIndex == 0 && upperIndex == lowerIndex) {
//         r1 = 0;
//         p1 = 0; 
//     } 

//     // If we reach past the end of the vector, return the last value 
//     if (upperIndex == size - 1 && upperIndex == lowerIndex) {
//         return p2; 
//     } 

//     double slope = (p2 - p1) / (r2 - r1); 
//     double deltaRadius = radius - r1; 
//     // mx + b 
//     // return slope * deltaRadius + p1; 
//     return p1 + slope * deltaRadius;
// }
