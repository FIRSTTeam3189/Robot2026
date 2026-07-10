#include "simulation/GamePiece.h"

namespace simulation::pieces {
    GamePiece::GamePiece(PieceInfo pieceInfo, b2World* physicsWorld, frc::Pose3d initialPose) 
    : m_pose(initialPose)
    , m_pieceInfo(std::move(pieceInfo))
    , m_state(GamePieceState::Airborne)
    , m_velocity(frc::Translation2d(0.0_m, 0.0_m))
    , m_physicsWorld(physicsWorld)
    {
        m_type = m_pieceInfo.type;

        m_body = physicsWorld->CreateBody(&m_pieceInfo.bodyDef);
        b2FixtureDef pieceFixture;
        pieceFixture.shape = m_pieceInfo.shape.get();
        pieceFixture.density = 1.0f;
        pieceFixture.friction = 0.5f;
        pieceFixture.restitution = pieceInfo.restitution;
        m_body->CreateFixture(&pieceFixture);
    }

    void GamePiece::SimulationSubTick(int subtickNum) {
        switch (m_state) {
            case GamePieceState::Airborne:
                for (int i = 0; i < constants::simulation::simulated_arena::kSubTicks; i++) 
                    AirborneStep(constants::simulation::simulated_arena::kSubDeltaTime);

                break;

            case GamePieceState::Grounded:
                SyncFromBody();
                break;
        }
    }

    void GamePiece::AirborneStep(units::second_t deltaTime) {
        if (m_state != GamePieceState::Airborne) {
            return;
        }

        // Apply gravity to vertical velocity
        m_velocity = frc::Translation3d(
            m_velocity.X(),
            m_velocity.Y(),
            m_velocity.Z() + units::meter_t{GRAVITY.value() * deltaTime.value()}
        );

        // Apply velocity to pose 
        frc::Translation3d nextStep {
            m_pose.X() + units::meter_t(m_velocity.X().value() * deltaTime.value()),
            m_pose.Y() + units::meter_t(m_velocity.Y().value() * deltaTime.value()),
            m_pose.Z() + units::meter_t(m_velocity.Z().value() * deltaTime.value())
        };

        // Check ground collision
        if (nextStep.Z() <= m_pieceInfo.height) {
            Land(frc::Translation3d(nextStep.X(), nextStep.Y(), m_pieceInfo.height));
            return;
        }

        m_pose = frc::Pose3d(nextStep, frc::Rotation3d());
    }

    void GamePiece::SyncFromBody() {
        b2Vec2 pos = m_body->GetPosition();
        float angle = m_body->GetAngle();

        m_pose = frc::Pose3d{
            frc::Translation3d{
                units::meter_t{pos.x},
                units::meter_t{pos.y},
                m_pieceInfo.height
            },
            frc::Rotation3d{0_rad, 0_rad, units::radian_t{angle}}
        };
    }

    void GamePiece::SyncToBody() {
        m_body->SetTransform(
            b2Vec2{
                (float)m_pose.X().value(),
                (float)m_pose.Y().value()
            },
            (float)m_pose.Rotation().Z().value()
        );

        m_body->SetLinearVelocity(
            b2Vec2{(float)m_velocity.X().value(), (float)m_velocity.Y().value()}
        );
        
        m_body->SetAngularVelocity(0.0f);
    }

    void GamePiece::Launch(const LaunchInfo& config) {
        /*
                /|
               / |
              /  |
             /   |
            /____|
            Remember your geometry...... given the angle is
            on the bottom left and distance (hypoetnuse) = 1, if we want to find
            the bottom leg/horizontal distance we can use cos(). if we want to find
            the right leg/vertical distance we can use sin(). Then we can scale it by
            the shooter speed :^)  
        */

        double horizontalSpeed = config.launchSpeed.value() * std::cos(config.shooterAngle.value());
        double verticalSpeed = config.launchSpeed.value() * std::sin(config.shooterAngle.value());;
    
        // XY velocity = chassis velocity + shooter-relative horizontal velocity
        // (mirrors calculateInitialProjectileVelocityMPS)
        double vx = config.chassisSpeedsFieldRelative.vx.value() 
                    + horizontalSpeed * std::cos(config.shooterFacing.Radians().value());
        double vy = config.chassisSpeedsFieldRelative.vy.value() 
                    + horizontalSpeed * std::sin(config.shooterFacing.Radians().value());

        m_velocity = frc::Translation3d{
            units::meter_t{vx},
            units::meter_t{vy},
            units::meter_t{verticalSpeed}
        };

        m_pose = frc::Pose3d{
            frc::Translation3d{
                units::meter_t{m_pose.X()},
                units::meter_t{m_pose.Y()},
                config.initialHeight
            },
            frc::Rotation3d{0_rad, -config.shooterAngle, config.shooterFacing.Radians()}
        };

        m_body->SetEnabled(false);
        m_state = GamePieceState::Airborne;
    }

    void GamePiece::Land(frc::Translation3d groundPosition) {
        m_pose = frc::Pose3d(groundPosition, frc::Rotation3d());
        SyncToBody();

        m_body->SetEnabled(true);
        m_state = GamePieceState::Grounded;
    }

    frc::Translation2d GamePiece::CalculateInitialProjectileVelocity(
        frc::Translation2d  shooterPositionOnRobot,
        frc::ChassisSpeeds  robotChassisSpeeds,
        frc::Rotation2d     chassisFacing,
        double              groundSpeedMetersPerSecond
    ) {
        /*
            Even if a robot isn't moving linearly, the spinning rotation
            of the robot can still affect the ball's initial velocity.
            Things farther away from the center of the robot spin faster
            than things closer to the center. This is called tangential velocity
            and we calculate it below for the shooter, as the shooter
            is usually not dead-center on the robot
        */
        frc::Translation2d shooterTangentialVelocity = shooterPositionOnRobot
            .RotateBy(chassisFacing)
            .RotateBy(frc::Rotation2d{90_deg})
            * robotChassisSpeeds.omega.value();

        const frc::Translation2d chassisLinearVelocity = frc::Translation2d(
            units::meter_t{robotChassisSpeeds.vx.value()},
            units::meter_t{robotChassisSpeeds.vy.value()}
        );

        frc::Translation2d shooterGroundVelocity = 
            chassisLinearVelocity + shooterTangentialVelocity;

        return shooterGroundVelocity + frc::Translation2d{
            units::meter_t{groundSpeedMetersPerSecond}, 
            chassisFacing
        };
    }

    frc::Translation3d GamePiece::GetVelocityInMeters() const {
        if (m_state == GamePieceState::Grounded) {
            // return m_velocity;
            b2Vec2 velocity = m_body->GetLinearVelocity();
            return frc::Translation3d(
                units::meter_t{velocity.x},
                units::meter_t{velocity.y},
                0_m
            );
        }

        return m_velocity;
    }
 
    std::string GamePiece::GetType() const {
        return m_type;
    }

    frc::Pose3d GamePiece::GetPose() const {
        return m_pose;
    }

    b2Body* GamePiece::GetBody() const {
        return m_body;
    }
}