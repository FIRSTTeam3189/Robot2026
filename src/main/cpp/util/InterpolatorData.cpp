#include "util/InterpolatorData.h"


InterpolatorData::InterpolatorData(const std::string& file)
    : m_filename(file)
{
    json j = ReadFromFile();

    if(j.contains("angles") && j.contains("powers")){
        m_angles = j["angles"].get<std::vector<double>>();
        m_powers = j["powers"].get<std::vector<double>>();
    }
}
 
void InterpolatorData::AddPoint(double angle, double power) {
    m_angles.push_back(angle);
    m_powers.push_back(power);
}

void InterpolatorData::RemoveLastPoint() {
    if (!m_angles.empty()) {
        m_angles.pop_back();
        m_powers.pop_back();
    }
}

void InterpolatorData::SaveToFile() {
    json j = ReadFromFile();

    std::vector<double> savedAngles = GetAnglesList(j);
    std::vector<double> savedPowers = GetPowersList(j); 

    m_angles.insert(m_angles.end(), std::begin(savedAngles), std::end(savedAngles));
    m_powers.insert(m_powers.end(), std::begin(savedPowers), std::end(savedPowers)); 

    j["angles"] = m_angles;
    j["powers"] = m_powers; 

    std::ofstream file(m_filename);

    file << j.dump(4);
    file.close();
}

json InterpolatorData::ReadFromFile() {
    std::ifstream file(m_filename);

    if (!file.is_open()) {
        return json{
            {"angles", json::array()},
            {"powers", json::array()}
        }; 
    }

    json j;

    file >> j;
    file.close();

    return j;
}

std::vector<double> InterpolatorData::GetAnglesList(json j) {
    if(j.contains("angles"))
        return j["angles"].get<std::vector<double>>();

    return {};
}

std::vector<double> InterpolatorData::GetPowersList(json j) {
    if(j.contains("powers"))
        return j["powers"].get<std::vector<double>>();

    return {};
}  