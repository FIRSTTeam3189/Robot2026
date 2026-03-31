#pragma once

#include <units/time.h>

namespace constants {
    namespace simulation {
        constexpr int kSubTicks = 5;
        constexpr units::second_t kLoopPeriod = 20_ms;
        constexpr units::second_t kSubDeltaTime = kLoopPeriod / kSubTicks;
    }
}