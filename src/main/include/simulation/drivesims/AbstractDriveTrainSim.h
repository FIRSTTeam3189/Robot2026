#pragma once

#include <array>

#include "units/mass.h"
#include "units/length.h"

#include "simulation/drivesims/SwerveModuleSimulation.h"


namespace simulation::drivesims {
    struct DriveTrainSimulationConfig {
        units::kilogram_t   robotMass;
        units::meter_t      bumperLengthX;
        units::meter_t      bumperWidthY;
        units::meter_t      trackLengthX;
        units::meter_t      trackWidthY;
        std::array<SwerveModuleSimulation, 4> swerveModules;
    };

    class AbstractDriveTrainSim {
    public:
        


    private:


    };
}