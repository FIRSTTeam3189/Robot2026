#pragma once

#include <memory>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <units/velocity.h>
#include <units/length.h>
#include <units/mass.h>

#include "simulation/box2d/box2d.h"
#include "constants/simulation/SimulatedArena.h"


namespace simulation::pieces {
    enum class GamePieceState {
        Airborne,
        Grounded,
        Held // Called when the robot holds it in its intake
    };

    struct PieceInfo {
        std::unique_ptr<b2Shape>    shape;
        const b2BodyDef             bodyDef;
        const std::string           type;
        const units::meter_t        height;
        const units::kilogram_t     mass;
        const double                restitution;
    };
    
    struct LaunchInfo {
        frc::Translation2d          shooterPositionOnRobot;
        frc::ChassisSpeeds          chassisSpeedsFieldRelative;
        frc::Rotation2d             shooterFacing;
        units::meter_t              initialHeight;
        units::meters_per_second_t  launchSpeed;
        units::radian_t             shooterAngle;
    };

    class GamePiece {
    public:
        GamePiece() = default;
        GamePiece(PieceInfo pieceInfo, b2World* physicsWorld, frc::Pose3d initialPose);
        virtual ~GamePiece() = default;

        virtual void SimulationSubTick(int subtickNum);
        virtual void AirborneStep(units::second_t deltaTime);

        void Launch(const LaunchInfo& config);
        void Land(frc::Translation3d groundPosition);

        void SyncFromBody();
        void SyncToBody();

        frc::Translation3d GetVelocityInMeters() const;
        std::string GetType() const;
        frc::Pose3d GetPose() const;
        b2Body* GetBody() const;


    private:
        frc::Translation2d CalculateInitialProjectileVelocity(
            frc::Translation2d  shooterPositionOnRobot,
            frc::ChassisSpeeds  robotChassisSpeeds,
            frc::Rotation2d     chassisFacing,
            double              groundSpeedMetersPerSecond
        );

        frc::Pose3d m_pose;
        b2World* m_physicsWorld;
        b2Body* m_body;
        std::string m_type;
        GamePieceState m_state;
        frc::Translation3d m_velocity;

        PieceInfo m_pieceInfo;

        constexpr static units::meter_t GRAVITY{ -9.81_m };
    };
};