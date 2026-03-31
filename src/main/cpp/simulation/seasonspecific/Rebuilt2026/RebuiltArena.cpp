#include "simulation/seasonspecific/Rebuilt2026/RebuiltArena.h"
#include "simulation/seasonspecific/Rebuilt2026/RebuiltFuel.h"


namespace simulation::arenas {
    RebuiltArena::RebuiltArena()
    : SimulatedArena(GetObstacleMap())
    , m_fuelPosePublisher(nt::NetworkTableInstance::GetDefault()
        .GetStructArrayTopic<frc::Pose3d>("/SmartDashboard/FieldSimulation/Fuel")
        .Publish())
    {
    }
    
    void RebuiltArena::PlaceGamePiecesOnField() {
        for (int x = 0; x < 12; x += 1) {
            for (int y = 0; y < 30; y++) {
                auto newFuel = std::make_unique<pieces::RebuiltFuel>(
                    m_physicsWorld,
                    frc::Pose3d(
                        centerPieceBottomRightCorner 
                        + frc::Translation3d(5.991_in * x, 5.95_in * y, 0_in),
                        frc::Rotation3d()
                    )
                );

                AddGamePiece(std::move(newFuel));
            }
        }
    }

    FieldMap RebuiltArena::GetObstacleMap() {
        FieldMap map;

        constexpr units::meter_t FIELD_X_MIN                     { 0.0_m };
        constexpr units::meter_t FIELD_X_MAX                     { 16.54105_m };
        constexpr units::meter_t FIELD_Y_MIN                     { 0.0_m };
        constexpr units::meter_t FIELD_Y_MAX                     { 8.06926_m };

        constexpr units::meter_t HUB_Y_LEN                       { 1.19380_m };
        constexpr units::meter_t HUB_X_LEN                       { 1.19380_m };
        constexpr units::meter_t HUB_X                           { 4.625594_m };
        constexpr units::meter_t HUB_Y                           { 4.03463_m };
        // 73 inches -> meters
        constexpr units::meter_t HUB_RAMP_LENGTH                 { 73.0_in };

        // Uprights and trench wall sizes (inches -> meters)
        constexpr units::meter_t UPRIGHT_X_LEN                   { 3.5_in };
        constexpr units::meter_t UPRIGHT_Y_LEN                   { 1.5_in };
        constexpr units::meter_t UPRIGHT_OFFSET_FROM_END_WALL    { 1.06204_m };
        constexpr units::meter_t UPRIGHT_OFFSET_FROM_SIDE_WALL   { 3.31524_m };
        constexpr units::meter_t UPRIGHT_Y_SPACING               { 33.75_in };

        constexpr units::meter_t TRENCH_WALL_Y_LEN               { 12.0_in };
        constexpr units::meter_t TRENCH_WALL_X_LEN               { 47.0_in };
        constexpr units::meter_t TRENCH_WALL_OFFSET_FROM_END_WALL{ 4.61769_m };
        constexpr units::meter_t TRENCH_WALL_OFFSET_FROM_SIDE_WALL{ 1.43113_m };
        
        map.AddBorderLine(
            frc::Translation2d(FIELD_X_MIN, FIELD_Y_MIN),
            frc::Translation2d(FIELD_X_MIN, FIELD_Y_MAX)
        );

        // red wall
        map.AddBorderLine(
            frc::Translation2d(FIELD_X_MAX, FIELD_Y_MIN), 
            frc::Translation2d(FIELD_X_MAX, FIELD_Y_MAX)
        );

        // scoring area walls
        map.AddBorderLine(
            frc::Translation2d(FIELD_X_MIN, FIELD_Y_MIN), 
            frc::Translation2d(FIELD_X_MAX, FIELD_Y_MIN)
        );

        // opposite side
        map.AddBorderLine(
            frc::Translation2d(FIELD_X_MIN, FIELD_Y_MAX), 
            frc::Translation2d(FIELD_X_MAX, FIELD_Y_MAX)
        );


        // blue ladder uprights
        map.AddRectangularObstacle(
            UPRIGHT_X_LEN.value(),
            UPRIGHT_Y_LEN.value(),
            frc::Pose2d(
                UPRIGHT_OFFSET_FROM_END_WALL, 
                UPRIGHT_OFFSET_FROM_SIDE_WALL, 
                frc::Rotation2d()
            )
        );
        
        map.AddRectangularObstacle(
            UPRIGHT_X_LEN.value(),
            UPRIGHT_Y_LEN.value(),
            frc::Pose2d(
                UPRIGHT_OFFSET_FROM_END_WALL,
                UPRIGHT_OFFSET_FROM_SIDE_WALL - UPRIGHT_Y_SPACING,
                frc::Rotation2d()
            )
        );

        // red ladder uprights
        map.AddRectangularObstacle(
            UPRIGHT_X_LEN.value(),
            UPRIGHT_Y_LEN.value(),
            frc::Pose2d(
                FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL, 
                FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL, 
                frc::Rotation2d()
            )
        );
        
        map.AddRectangularObstacle(
            UPRIGHT_X_LEN.value(),
            UPRIGHT_Y_LEN.value(),
            frc::Pose2d(
                FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
                FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL + UPRIGHT_Y_SPACING,
                frc::Rotation2d()
            )
        );

        // red trench wall
        map.AddRectangularObstacle(
            TRENCH_WALL_X_LEN.value(),
            TRENCH_WALL_Y_LEN.value(),
            frc::Pose2d(
                FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
                TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
                frc::Rotation2d()
            )
        );

        map.AddRectangularObstacle(
            TRENCH_WALL_X_LEN.value(),
            TRENCH_WALL_Y_LEN.value(),
            frc::Pose2d(
                FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
                FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
                frc::Rotation2d()
            )
        );

        // blue hub
        map.AddRectangularObstacle(
            HUB_X_LEN.value(), 
            HUB_Y_LEN.value(), 
            frc::Pose2d(
                HUB_X, 
                HUB_Y, 
                frc::Rotation2d()
            )
        );

        // red hub
        map.AddRectangularObstacle(
            HUB_X_LEN.value(), 
            HUB_Y_LEN.value(), 
            frc::Pose2d(
                FIELD_X_MAX - HUB_X, 
                HUB_Y, 
                frc::Rotation2d()
            )
        );

        return map;
    }

    void RebuiltArena::SimulationPeriodic() {
        SimulatedArena::SimulationPeriodic();

        PublishNetworkTables();
    }

    void RebuiltArena::PublishNetworkTables() {
        auto fuelPieces = GetInstance().GetGamePiecesByType("Fuel");
        if (!fuelPieces.empty()) {
            const pieces::GamePiece* first = fuelPieces[0];
        }

        auto fuelPoses = GetInstance().GetGamePiecesPosesByType("Fuel");
        m_fuelPosePublisher.Set(fuelPoses);
    }
}