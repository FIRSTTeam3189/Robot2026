#include "util/interpolatorDataUtils/InterpolatorData.h"

InterpolatorData::InterpolatorData(const std::string &filename)
    : m_filename(filename)

{
    json j = ReadFromFile();
    bool readFile = false; 
    if (j.contains("shots"))
    {
        readFile = true; 
        m_dataPoints = GetDataPointsList(j);
    }

    frc::SmartDashboard::PutBoolean("Interpolator Data/file read", readFile); 

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

    // not actually needed 
    // json j = ReadFromFile();
    // std::vector<Shotpoint> savedDataPoints = GetDataPointsList(j);
    // savedDataPoints.insert(savedDataPoints.end(), m_dataPoints.begin(), m_dataPoints.end()));
    json j; 
    j["shots"] = json::array();
    for (const Shotpoint& shot : m_dataPoints) {

        json shotJson = shot.ToJson();
        j["shots"].push_back(shotJson);

    }

    std::ofstream file(m_filename);

    if (file.is_open()) {
        file << j.dump(4);
        file.close();
    }
}

json InterpolatorData::ReadFromFile()
{
    namespace fs = std::filesystem; 

    std::ifstream file(m_filename);
    if (!file.is_open() || fs::is_empty(m_filename))
    {
        return json{
            {"shots", json::array()}
        };
    }

    json j;
    file >> j;

    file.close();
    fs::remove(m_filename); 

    return j;
}

std::vector<Shotpoint> InterpolatorData::GetDataPointsList(json j)
{
    std::vector<Shotpoint> shotpoints; 

    for (json shot : j["shots"]) {
        shotpoints.push_back(Shotpoint::ToShotpoint(shot)); 
    }

    return shotpoints;
}