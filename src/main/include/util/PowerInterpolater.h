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


class PowerInterpolater {
    public: 
        PowerInterpolater(std::vector<units::meter_t> distances, std::vector<double> powers);
        double GetPower(units::meter_t radius);
    private:    
        std::vector<units::meter_t> m_distances; 
        std::vector<double> m_powers; 

};