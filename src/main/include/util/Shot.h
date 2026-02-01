#pragma once

#include "util/InterpolatorData.h"

struct Shotpoint {
    frc::Pose2d robotPose;
    double power;
    double hoodAngle;
    double robotToHubDistance;
};

// j["distance"]