#pragma once

#include <units/time.h>
#include <units/frequency.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <cmath> 
#include <vector>
#include <algorithm>  

#include "util/Vector2d.h"
#include <frc/smartdashboard/SmartDashboard.h>



namespace util {
    namespace interpolator {
        class Linear {
            public: 
                Linear(std::vector<double> values1, std::vector<double> values2);
                double Predict(double value1); 
            private:    
                std::vector<double> m_values1; 
                std::vector<double> m_values2; 

        };
    }
}