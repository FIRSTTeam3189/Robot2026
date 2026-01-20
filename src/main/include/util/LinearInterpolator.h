#pragma once

#include <cmath> 
#include <vector>
#include <algorithm>  
#include <functional> 

enum EndType {
    END_LIMIT,
    CONTINIOUS
};

template <typename P, typename K>
class LinearInterpolator {
    public: 
        LinearInterpolator(const std::vector<P> &xValues, const std::vector<K> &yValues); 
        K Predict(P x); 
    private:
        std::vector<P> m_xValues;
        std::vector<K> m_yValues; 
        auto m_PToDouble;
        auto m_KToDouble; 
};