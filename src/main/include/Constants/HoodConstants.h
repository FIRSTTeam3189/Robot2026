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

namespace HoodConstants {
    constexpr units::meter_t kHoodHeight {0.0};
    constexpr units::degree_t kHoodDegreeOffset {2.0_deg}; 

};
