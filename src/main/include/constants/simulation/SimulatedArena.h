#pragma once

#include <units/time.h>

namespace constants::simulation::simulated_arena {
    constexpr int kSubTicks = 5;
    constexpr units::second_t kLoopPeriod = 20_ms;
    constexpr units::second_t kSubDeltaTime = kLoopPeriod / kSubTicks;
}