#include "util/interpolatorDataUtils/Shotpoint.h"



Shotpoint::Shotpoint(frc::Pose2d robotPose, double power, double hoodangle, double robotToHubDistance)
  :
    m_robotPose(robotPose),
    m_power(power),
    m_hoodAngle(hoodangle),
    m_robotToHubDistance(robotToHubDistance)
{

}

json Shotpoint::ToJson() const {

    json shot;

    shot["pose"]["x"] = m_robotPose.X().value(); 
    shot["pose"]["y"] = m_robotPose.Y().value();
    shot["pose"]["rot"] = m_robotPose.Rotation().Degrees().value();

    shot["distance"] = m_robotToHubDistance;
    shot["hoodAngle"] = m_hoodAngle;
    shot["power"] = m_power;

    return shot;
}

Shotpoint Shotpoint::ToShotpoint(json shot) {
    units::meter_t x = units::meter_t{shot["pose"]["x"].get<double>()};
    units::meter_t y = units::meter_t{shot["pose"]["y"].get<double>()};
    units::degree_t rotAngle = units::degree_t{shot["pose"]["rot"].get<double>()}; 

    frc::Pose2d robotPose {x, y, frc::Rotation2d(rotAngle)};

    double distance = shot["distance"].get<double>(); 
    double hoodAngle = shot["hoodAngle"].get<double>();     
    double power = shot["power"].get<double>(); 

    return {robotPose, power, hoodAngle, distance}; 
}
