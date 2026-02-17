#pragma once 

#include <fstream>
#include <string>
#include <vector>
#include <filesystem>

#include <frc/smartdashboard/SmartDashboard.h>

#include "util/interpolatorDataUtils/Shotpoint.h"

using json = nlohmann::json;

class InterpolatorData {

    public:

        explicit InterpolatorData(const std::string& filename);

        void AddPoint(frc::Pose2d robotPose, units::meter_t distanceToTarget, units::degree_t hoodAngle, double flywheelPower);

        void RemoveLastPoint();

        void SaveToFile();

        json ReadFromFile();

        std::vector<Shotpoint> GetDataPointsList(json j);

    private:

        const std::string m_filename;

        std::vector<Shotpoint> m_dataPoints; 

};