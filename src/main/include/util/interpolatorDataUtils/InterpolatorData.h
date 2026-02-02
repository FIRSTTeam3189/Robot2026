#pragma once 

#include <fstream>
#include <string>
#include <vector>

#include "util/interpolatorDataUtils/Shotpoint.h"
#include "util/Vector2d.h"

using json = nlohmann::json;


class InterpolatorData {

    public:

        explicit InterpolatorData(const std::string& file);

        void AddPoint(frc::Pose2d robotPose, frc::Pose2d targetPose, units::degree_t hoodAngle, double flywheelPower);

        void RemoveLastPoint();

        void SaveToFile();

        json ReadFromFile();

        std::vector<Shotpoint> GetDataPointsList(json j);

    private:

        const std::string m_filename;

        std::vector<Shotpoint> m_dataPoints; 

};