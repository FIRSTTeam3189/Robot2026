#pragma once 

#include "json/json.hpp"
#include <fstream>
#include <string>
#include <vector>

#include "subsystems/shooterSubsystem/Flywheel.h"
#include "subsystems/shooterSubsystem/Articulation.h"
#include "subsystems/SwerveDrive.h"


using json = nlohmann::json;


class InterpolatorData {

    public:

        explicit InterpolatorData(const std::string& file);

        void AddPoint(double angle, double power);

        void RemoveLastPoint();

        void SaveToFile();

        json ReadFromFile();

        std::vector<double> GetAnglesList(json j);

        std::vector<double> GetPowersList(json j);

    private:

        const std::string m_filename;

        std::vector<double> m_angles;
        std::vector<double> m_powers;

};