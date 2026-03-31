#pragma once

/*
    DESIGN NOTES
    This class follows a C++ Singleton pattern, in which any
    file can require this class and obtain the same
    object. It ensures that only one simulated arena
    is active in a project while also being global to 
    the entire codebase.

    ts is so buns but no one is gonna notice ;)
*/

#include <iostream>
#include <units/length.h>

#include <networktables/StructArrayTopic.h>

#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>

#include "simulation/Simulatable.h"
#include "simulation/FieldMap.h"
#include "simulation/GamePiece.h"
#include "simulation/box2d/box2d.h"


namespace simulation {
    class SimulatedArena {
    public:
        /**
         * Gets the project's current SimulatedArena.
         * If there is none specified when first called,
         * it will default to the current season's arena.
         * 
         * @return SimulatedArena
         */
        static SimulatedArena& GetInstance();

        /**
         * Sets the project's SimulatedArena and overrides
         * the value returned by `GetInstance`. This allows
         * one to simulate a season's arena that's different 
         * from the current one.
         */
        virtual void PlaceGamePiecesOnField() = 0;

        /**
         * Runs once every simulation tick. It is essential that if
         * you create a simulated arena and override SimulationPeriodic,
         * you have to call the Base class method first
         */
        virtual void SimulationPeriodic();

        /**
         * Returns the collision shapes of the field as a new FieldMap
         * object. This is where one should define the solid areas of
         * a field to pass into the physics world.
         * 
         * @return FieldMap
         */
        virtual FieldMap GetObstacleMap() = 0;

        /** 
         * Returns the score of a given alliance 
         * 
         * @param alliance The alliance to get the score on
         * @return int
         */
        int GetScore(frc::DriverStation::Alliance alliance);

        /** 
         * Adds the score of an alliance by a given amount
         *  
         * @param alliance The alliance to give points to
         * @param pointsToAdd The amount of points to add by
         */
        void AddScore(frc::DriverStation::Alliance alliance, int pointsToAdd);

        /**
         * 
         */
        void AddDriveTrainSimulation();
        
        /**
         * Adds the given game piece to the current simulation
         * 
         * @param gamePiece A unique pointer to the game piece you want to add
         */
        void AddGamePiece(std::unique_ptr<pieces::GamePiece> gamePiece);

        /**
         * Removes the given game piece from the current simulation
         * 
         * @param gamePiece A raw pointer to the game piece oyu want to remove
         */
        void RemoveGamePiece(pieces::GamePiece* gamePiece);

        /**
         * Returns a vector of the poses of all of the game pieces with the given type
         * 
         * @param type The type of game piece to filter through
         * @return std::vector<frc::Pose3d>
         */
        std::vector<frc::Pose3d> GetGamePiecesPosesByType(const std::string& type);

        /**
         * Returns a vector of all of the game pieces with the given type
         * 
         * @param type The type of game piece to filter through
         * @return std::vector<GamePiece*>
         */
        std::vector<pieces::GamePiece*> GetGamePiecesByType(const std::string& type);
        virtual void PublishNetworkTables() = 0;

        void AddSimulatable(Simulatable& simulatable);

        ~SimulatedArena();

        // Prevents ways of creating base class like:
        // SimulatedArena arena;
        // SimulatedArena arenaB = arenaA;
        SimulatedArena(const SimulatedArena&) = delete;
        SimulatedArena& operator=(const SimulatedArena&) = delete;
        
    protected:
        // SimulatedArena is not supposed to be instantiated outside,
        // but its constructor can be called in subclasses, which means
        // it gets put in protected :)

        /**
         * This constructor is only used for derived classes in its
         * initializor list. If you want to obtain a SimulatedArena,
         * call GetInstance() instead.
         */
        SimulatedArena(FieldMap obstacleMap);
        b2World* m_physicsWorld;
        FieldMap m_obstacleMap;
        
    private:
        static SimulatedArena* s_instance;
        
        int m_redScore{ 0 };
        int m_blueScore{ 0 };
        
        frc::Timer m_matchClock;
        std::vector<simulation::Simulatable*> m_simulatables;
        std::vector<std::unique_ptr<pieces::GamePiece>> m_gamePieces;
    };
}