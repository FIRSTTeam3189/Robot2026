#pragma once

#include "simulation/SimulatedArena.h"
namespace simulation::maps {
    struct RebuiltObstaclesFieldMap : public FieldMap {
    private:
        // FIELD CONSTANTS
        static constexpr units::meter_t FIELD_X_MIN                     { 0.0_m };
        static constexpr units::meter_t FIELD_X_MAX                     { 16.54105_m };
        static constexpr units::meter_t FIELD_Y_MIN                     { 0.0_m };
        static constexpr units::meter_t FIELD_Y_MAX                     { 8.06926_m };

        static constexpr units::meter_t HUB_X_LEN                       { 1.19380_m };
        static constexpr units::meter_t HUB_Y_LEN                       { 1.19380_m };
        static constexpr units::meter_t HUB_X                           { 4.625594_m };
        static constexpr units::meter_t HUB_Y                           { 4.03463_m };
        // 73 inches -> meters
        static constexpr units::meter_t HUB_RAMP_LENGTH                 { 73.0_in };

        // Uprights and trench wall sizes (inches -> meters)
        static constexpr units::meter_t UPRIGHT_X_LEN                   { 3.5_in };
        static constexpr units::meter_t UPRIGHT_Y_LEN                   { 1.5_in };
        static constexpr units::meter_t UPRIGHT_OFFSET_FROM_END_WALL    { 1.06204_m };
        static constexpr units::meter_t UPRIGHT_OFFSET_FROM_SIDE_WALL   { 3.31524_m };
        static constexpr units::meter_t UPRIGHT_Y_SPACING               { 33.75_in };

        static constexpr units::meter_t TRENCH_WALL_Y_LEN               { 12.0_in };
        static constexpr units::meter_t TRENCH_WALL_X_LEN               { 47.0_in };
        static constexpr units::meter_t TRENCH_WALL_OFFSET_FROM_END_WALL{ 4.61769_m };
        static constexpr units::meter_t TRENCH_WALL_OFFSET_FROM_SIDE_WALL{ 1.43113_m };

    public:
        RebuiltObstaclesFieldMap();
    };
};

namespace simulation::arenas {
    class RebuiltArena : public simulation::SimulatedArena {
    public:
        RebuiltArena();
        void SimulationPeriodic() override; 
        void PlaceGamePiecesOnField() override;
        void PublishNetworkTables() override;
        FieldMap GetObstacleMap() override;
        
    protected:
        constexpr static frc::Translation3d centerPieceBottomRightCorner{7.35737_m, 1.724406_m, 0.0_m};
        constexpr static frc::Translation3d redDepotBottomRightCorner   {0.02_m, 5.53_m, 0_m};
        constexpr static frc::Translation3d blueDepotBottomRightCorner  {16.0274_m, 1.646936_m, 0.0_m};

    private:
        nt::StructArrayPublisher<frc::Pose3d> m_fuelPosePublisher;
    };
};
