#include "simulation/SimulatedArena.h"
#include "simulation/seasonspecific/Rebuilt2026/RebuiltArena.h"

#include "constants/Simulation.h"


namespace simulation {
    SimulatedArena::SimulatedArena(FieldMap obstacleMap) 
    : m_obstacleMap(std::move(obstacleMap))
    , m_simulatables()
    , m_matchClock()
    , m_physicsWorld(new b2World(b2Vec2(0.0f, 0.0f)))
    {
        // Create bodies
        for (const auto& obstacle : m_obstacleMap.obstacles) {
            b2FixtureDef obstacleFixtureDef;
            obstacleFixtureDef.shape = obstacle.shape.get();
            obstacleFixtureDef.density = 1;
            obstacleFixtureDef.restitution = 0.5;

            b2Body* obstacleBody = m_physicsWorld->CreateBody(&obstacle.bodyDef);
            obstacleBody->CreateFixture(&obstacleFixtureDef);
        }
    }

    SimulatedArena& SimulatedArena::GetInstance() {
        if (frc::RobotBase::IsReal()) {
            throw std::runtime_error("Cannot reference the SimulatedArena when running on the real robot.");
        }

        static simulation::arenas::RebuiltArena instance;
        return instance;
    }

    void SimulatedArena::SimulationPeriodic() {
        m_physicsWorld->Step(constants::simulation::kLoopPeriod.value(), 4, 2);

        for (auto& gamePiece : m_gamePieces) { 
            gamePiece->SimulationPeriodic();
        }
    }
    
    int SimulatedArena::GetScore(frc::DriverStation::Alliance alliance) {
        switch (alliance) {
            case frc::DriverStation::Alliance::kRed     : return m_redScore;
            case frc::DriverStation::Alliance::kBlue    : return m_blueScore;
            default: throw std::runtime_error("Specified alliance does not exist");
        }
    }
    
    void SimulatedArena::AddScore(frc::DriverStation::Alliance alliance, int pointsToAdd) {
        switch (alliance) {
            case frc::DriverStation::Alliance::kRed:
            m_redScore += pointsToAdd;
            break;
            
            case frc::DriverStation::Alliance::kBlue: 
            m_blueScore += pointsToAdd;
            break;
        }
    }
    
    void SimulatedArena::AddGamePiece(std::unique_ptr<pieces::GamePiece> gamePiece) {
        m_gamePieces.emplace_back(std::move(gamePiece));
    }
    
    void SimulatedArena::RemoveGamePiece(pieces::GamePiece* gamePiece) {
        /*
        We cannot just use std::erase(m_gamePieces, gamePiece)
        as it would try and compare a unique_ptr with a raw pointer
        */
       std::erase_if(m_gamePieces, [gamePiece](const auto& ptr) {
           return ptr.get() == gamePiece;
        });
        
        m_physicsWorld->DestroyBody(gamePiece->GetBody());
    }
    
    std::vector<frc::Pose3d> SimulatedArena::GetGamePiecesPosesByType(const std::string& type) {
        std::vector<frc::Pose3d> poses;

        /*
            & = reference to the unique_ptr instead
            of trying to creating a copy, which it can't do
        */
        for (const auto& gamePiece : m_gamePieces) {
            if (gamePiece->GetType() == type) {
                poses.emplace_back(gamePiece->GetPose());
            }
        }

        return poses;
    }

    std::vector<pieces::GamePiece*> SimulatedArena::GetGamePiecesByType(const std::string& type) {
        std::vector<pieces::GamePiece*> gamePieces;

        for (const auto& gamePiece : m_gamePieces) {
            if (gamePiece->GetType() == type) {
                gamePieces.emplace_back(gamePiece.get());
            }
        }

        return gamePieces;
    }

    void SimulatedArena::PublishNetworkTables() {
        std::vector<frc::Pose3d> poses = GetInstance().GetGamePiecesPosesByType("Fuel");

    }

    void SimulatedArena::AddSimulatable(Simulatable& simulatable) {
        // m_simulatables.emplace_back(&simulatable);
    }

    SimulatedArena::~SimulatedArena() {
        delete m_physicsWorld;
    }
}

