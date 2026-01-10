#pragma once

#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/system/plant/DCMotor.h>

#define PI 3.14159265358979323846

namespace AutoConstants {
    // Distance from robot center to furthest module
    constexpr units::meters_per_second_t kMaxAutoModuleSpeed{4.0_mps}; // 5.0_mps

    // Translation PID
    constexpr double kPTranslationAuto {4.75};
    // constexpr double kPTranslationAuto {0.0};
    constexpr double kITranslationAuto {0.0};
    constexpr double kDTranslationAuto {0.0};

    // Rotation PID
    // constexpr double kPRotationAuto {0.0};
    constexpr double kPRotationAuto {5.0}; // 5.0
    constexpr double kIRotationAuto {0.0};
    constexpr double kDRotationAuto {0.0};


    constexpr units::volt_t kNominalVoltage {12.0_V};
    constexpr units::newton_meter_t kStallTorque = units::newton_meter_t{43.32};
    constexpr units::ampere_t kStallCurrent = units::ampere_t{366};
    constexpr units::ampere_t kFreeCurrent = units::ampere_t{2};
    constexpr units::radians_per_second_t kFreeSpeed = units::radians_per_second_t{102.8};
    constexpr units::kilogram_t kRobotMass {54.431_kg}; // TBD
    constexpr units::kilogram_square_meter_t kRobotMOI = units::kilogram_square_meter_t{6.750};

    // using namespace std::literals;
    // constexpr std::array kAutonomousPaths {
    //     "Test - Line"sv
    // };
}