#pragma once

#include "json/json.hpp"
#include <fstream>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>


using json = nlohmann::json;

struct Shotpoint {

    frc::Pose2d m_robotPose;
    double m_power;
    double m_hoodAngle;
    double m_robotToHubDistance;

    Shotpoint(frc::Pose2d robotPose, double power, double hoodangle, double robotToHubDistance);

    json ToJson() const;

    static Shotpoint ToShotpoint(json j); 
};