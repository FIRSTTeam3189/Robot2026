#include "util/interpolatorDataUtils/InterpolatorData.h"

InterpolatorData::InterpolatorData(const std::string &file)
    : m_filename(file)

{
    json j = ReadFromFile();
    if (j.contains("shots"))
    {
        m_dataPoints = GetDataPointsList(j);
    }

}

void InterpolatorData::AddPoint(frc::Pose2d robotPose, frc::Pose2d targetPose, units::degree_t hoodAngle, double flywheelPower)
{

    Vector2d robotToTarget(robotPose, targetPose); 
    double distance = robotToTarget.Length(); 

    m_dataPoints.push_back({robotPose, flywheelPower, hoodAngle.value(), distance});
}

void InterpolatorData::RemoveLastPoint()
{
    if (!m_dataPoints.empty())
    {
        m_dataPoints.pop_back();
    }

}

void InterpolatorData::SaveToFile()
{

    json j = ReadFromFile();
    std::vector<Shotpoint> savedDataPoints = GetDataPointsList(j);
    m_dataPoints.insert(m_dataPoints.end(), std::begin(savedDataPoints), std::end(savedDataPoints));

    j["shots"] = json::array();
    for (const auto& shot : m_dataPoints) {

        json shotJson = shot.ToJson();
        j["shots"].push_back(shotJson);

    }

    std::ofstream file(m_filename);
    file << j.dump(4);
    file.close();
}

json InterpolatorData::ReadFromFile()

{
    std::ifstream file(m_filename);
    if (!file.is_open())
    {
        return json{
            {"shots", json::array()}
        };
    }
    json j;
    file >> j;
    file.close();
    return j;

}


std::vector<Shotpoint> InterpolatorData::GetDataPointsList(json j)
{
    std::vector<Shotpoint> shotpoints; 

    for (json shot : j["shots"]) {
        shotpoints.push_back(Shotpoint::ToShotpoint(j)); 
    }

    return shotpoints;
}