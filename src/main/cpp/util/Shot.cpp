#include "util/Shot.h"

json toJson(const Shotpoint& shot) {
    json j;

    j["pose"]["x"] = shot.robotPose.X().value();
    j["pose"]["y"] = shot.robotPose.Y().value();
    j["pose"]["rot"] = shot.robotPose.Rotation().Degrees().value();

    j["distance"] = shot.robotToHubDistance;
    j["hood"] = shot.hoodAngle;
    j["power"] = shot.power;

    return j;
}