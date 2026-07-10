#include "simulation/drive/SwerveDriveSimulation.h"
#include "simulation/SimulatedArena.h"
#include "constants/Vision.h"

#include <units/velocity.h>

namespace simulation::drive {
    //--------------------- Public ---------------------

    /**
     * Creates a new swerve drive simulation instance.
     */
    SwerveDriveSimulation::SwerveDriveSimulation(Config config, frc::Pose2d initialPose)
    : m_config(config)
    , m_moduleTranslations(config.moduleTranslations)
    , m_kinematics(config.moduleTranslations)
    , m_poseEstimator(
        m_kinematics,
        m_gyroSimulation.GetGyroReading(),
        GetModulePositions(),
        frc::Pose2d{}

        // ! i don't thikn we even need this lol...
        ,
        constants::vision::kEncoderTrustCoefficients,
        constants::vision::kMainVisionTrustCoefficients
    )
    , m_modules(CreateModules(config))
    , m_moduleGravityForce(config.robotMass.value() * 9.8 / m_modules.size())
    , m_gyroSimulation(0.5_deg, 0.02)
    {
        // Creating physics world body
        b2BodyDef bodyDef;
        bodyDef.type = b2BodyType::b2_dynamicBody;
        bodyDef.position.Set(
            initialPose.X().value(), 
            initialPose.Y().value()
        );

        bodyDef.angle = initialPose.Rotation().Radians().value();
        
        b2World* physicsWorld = SimulatedArena::GetInstance().GetPhysicsWorld();
        m_body = physicsWorld->CreateBody(&bodyDef);
        
        b2PolygonShape bodyShape;
        bodyShape.SetAsBox(config.bumperLengthX.value() / 2, config.bumperWidthY.value() / 2);

        b2FixtureDef bodyFixtureDef;
        bodyFixtureDef.shape = &bodyShape;
        
        float bodyArea = config.bumperLengthX.value() * config.bumperWidthY.value();
        bodyFixtureDef.density = config.robotMass.value() / bodyArea;

        m_body->CreateFixture(&bodyFixtureDef);
    }

    SwerveDriveSimulation::~SwerveDriveSimulation() {
        b2World* physicsWorld = SimulatedArena::GetInstance().GetPhysicsWorld();
        physicsWorld->DestroyBody(m_body);
    }
    
    std::array<SwerveModuleSimulation, 4> SwerveDriveSimulation::CreateModules(SwerveDriveSimulation::Config config) {
        return {
            SwerveModuleSimulation(config.moduleConfigs[0]),
            SwerveModuleSimulation(config.moduleConfigs[1]),
            SwerveModuleSimulation(config.moduleConfigs[2]),
            SwerveModuleSimulation(config.moduleConfigs[3]),
        };
    }

    // Movement

    /**
     * Halts all robot movement by stopping the swerve modules.
     */
    void SwerveDriveSimulation::Stop() {
        for (auto& swerveModule : m_modules) {
            swerveModule.Stop();
        };
    }

    /**
     * Obtains the field-relative chassis speeds according to the
     * encoder readings. If you'd like to obtain the actual field-relative
     * chassis speeds according to the physics simulation, use
     * GetPhysicsRobotRelativeSpeeds()
     */
    frc::ChassisSpeeds SwerveDriveSimulation::GetRobotRelativeSpeeds() const {
        frc::SwerveModuleState frontLeftModuleState = m_modules[0].GetState();
        frc::SwerveModuleState frontRightModuleState = m_modules[1].GetState();
        frc::SwerveModuleState backLeftModuleState =  m_modules[2].GetState();
        frc::SwerveModuleState backRightModuleState =  m_modules[3].GetState();

        return m_kinematics.ToChassisSpeeds(
            frontLeftModuleState, frontRightModuleState, backLeftModuleState, backRightModuleState
        );
    };


    /**
     * Obtains the actual field-relative chassis speeds according to the physics
     * simulation. If you'd like to obtain the chassis speeds as read from
     * the encoder, use GetRobotRelativeSpeeds()
     */
    frc::ChassisSpeeds SwerveDriveSimulation::GetPhysicsRobotRelativeSpeeds() const {
        b2Vec2 linearVelocity = m_body->GetLinearVelocity();
        float angularVelocity = m_body->GetAngularVelocity();
        
        frc::ChassisSpeeds speeds(
            units::meters_per_second_t{ linearVelocity.x },
            units::meters_per_second_t{ linearVelocity.y },
            units::radians_per_second_t{ angularVelocity }
        );

        // Convert to relative
        speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            speeds, 
            GetPhysicsPose().Rotation()
        );

        return speeds;
    }

    /**
     * Returns the chassis speeds the modules are attempting to achieve. If the robot
     * maintains its current driving voltage and steering position for a long
     * enough period, it will achieve these speeds
     */
    frc::ChassisSpeeds SwerveDriveSimulation::GetDesiredSpeeds() const {
        frc::SwerveModuleState frontLeftModuleState = m_modules[0].GetEquilibriumState();
        frc::SwerveModuleState frontRightModuleState = m_modules[1].GetEquilibriumState();
        frc::SwerveModuleState backLeftModuleState =  m_modules[2].GetEquilibriumState();
        frc::SwerveModuleState backRightModuleState =  m_modules[3].GetEquilibriumState();

        return m_kinematics.ToChassisSpeeds(
            frontLeftModuleState, 
            frontRightModuleState, 
            backLeftModuleState, 
            backRightModuleState
        );
    };

    /**
     * 
     */
    void SwerveDriveSimulation::RunSwerveStates(std::array<frc::SwerveModuleState, 4> states) {
        for (int i = 0; SwerveModuleSimulation& moduleSimulation : m_modules) {
            states[i].Optimize(moduleSimulation.GetAngleMotorAbsoluteFacing());
            moduleSimulation.RunSwerveState(states[i]);

            ++i;
        };
    };

    void SwerveDriveSimulation::RunChassisSpeeds(frc::ChassisSpeeds speeds, frc::Translation2d center, bool fieldRelative) {
         if (fieldRelative) {
            speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    speeds, GetOdometryEstimatedPose().Rotation());
        }
        // if (discretizeSpeeds) {
        //     chassisSpeeds = ChassisSpeeds.discretize(
        //             chassisSpeeds,
        //             SimulatedArena.getSimulationDt().in(Seconds) * SimulatedArena.getSimulationSubTicksIn1Period());
        // }

        wpi::array<frc::SwerveModuleState, 4> setpoints = m_kinematics.ToSwerveModuleStates(speeds, center);
        RunSwerveStates(setpoints);
    };

    std::array<SwerveModuleSimulation, 4>& SwerveDriveSimulation::GetModules() {
        return m_modules;
    }

    // Positioning

    wpi::array<frc::SwerveModulePosition, 4> SwerveDriveSimulation::GetModulePositions() const {
        frc::SwerveModulePosition frontLeftPosition     = m_modules[0].GetPosition();
        frc::SwerveModulePosition frontRightPosition    = m_modules[1].GetPosition();
        frc::SwerveModulePosition backLeftPosition      = m_modules[2].GetPosition();
        frc::SwerveModulePosition backRightPosition     = m_modules[3].GetPosition();

        return wpi::array<frc::SwerveModulePosition, 4>{
            frontLeftPosition,
            frontRightPosition,
            backLeftPosition,
            backRightPosition
        };
    }

    /** 
     * Gets the robot's pose measured by odometry.
     * This estimation includes measurement errors that accumulate over time
     * due to skidding and odometry drift.
     * 
     * If you'd like to get the actual pose in the simulation, use
     * GetPhysicsPose() instead.
     */
    frc::Pose2d SwerveDriveSimulation::GetOdometryEstimatedPose() const {
        return m_poseEstimator.GetEstimatedPosition();
    };

    frc::Pose2d SwerveDriveSimulation::GetPhysicsPose() const {
        const b2Vec2 position = m_body->GetPosition(); 
        // std::cout << " Position: (" << position.x << ", " << position.y << ")" << std::endl;
        return frc::Pose2d(
            units::meter_t{position.x},
            units::meter_t{position.y},
            units::radian_t{m_body->GetAngle()}
        );
    };

    /**
     * Teleports the robot in the physics simulation to a certain
     * pose on the field. Does not sync with the stored pose
     * in the pose estimator (odometry).
     */
    void SwerveDriveSimulation::SetPhysicsPose(frc::Pose2d pose) {
        b2Vec2 position{ 
            (float)pose.X().value(), 
            (float)pose.Y().value() 
        };
        
        float angle{ (float)pose.Rotation().Radians().value() };

        m_body->SetTransform(position, angle);
        m_body->SetLinearVelocity(b2Vec2{0.0f, 0.0f});
    };

    /**
     * Resets the odometry to a certain passed position 
     * as frc::Pose2d.
     */
    void SwerveDriveSimulation::ResetOdometry(frc::Pose2d pose) {
        m_poseEstimator.ResetPosition(
            m_gyroSimulation.GetGyroReading(), 
            GetModulePositions(), 
            pose
        );
    };

    // Getters

    /**
     * Returns the maximum possible angular velocity
     */
    units::radians_per_second_t SwerveDriveSimulation::GetMaxAngularVelocity() const { 
        return units::radians_per_second_t{
            m_modules[0].GetMaxGroundSpeed().value()
            / m_moduleTranslations[0].Norm().value()
        };
    };


    /**
     * Returns the maximum possible angular acceleration
     */
    units::radians_per_second_squared_t SwerveDriveSimulation::GetMaxAngularAcceleration(units::ampere_t statorCurrentLimit) const {
        units::newton_t maxPropellingForcePerModule{ m_modules[0].GetMaxPropellingForce(
            m_config.robotMass,
            m_modules.size(),
            statorCurrentLimit,
            m_moduleGravityForce
        )};

        return units::radians_per_second_squared_t{
            maxPropellingForcePerModule.value()
            * m_moduleTranslations[0].Norm().value()
            * m_modules.size()
            / m_body->GetInertia()
        };
    };

    /**
     * Returns the gyro simulation of the swerve drive simulation.
     */
    GyroSimulation& SwerveDriveSimulation::GetGyroSimulation() {
        return m_gyroSimulation;
    }

    /**
     * Returns the swerve drive kinematics of the swerve drive simulation.
     */
    frc::SwerveDriveKinematics<4>& SwerveDriveSimulation::GetSwerveDriveKinematics() {
        return m_kinematics;
    }

    
    void SwerveDriveSimulation::SimulationSubTick(int subtickNum) {
        // Update simulations
        for (auto& moduleSimulation : m_modules) {
            moduleSimulation.SimulationSubTick();
        }

        // The physics that apply to the swerve drive
        // SimulateChassisFrictionForce();
        // SimulateChassisFrictionTorque();
        SimulateModulePropellingForces();

        units::radians_per_second_t currentAngularVelocity{ m_body->GetAngularVelocity() };
        m_gyroSimulation.SimulationSubTick(currentAngularVelocity);

        // Update pose estimator
        auto deltaTime = constants::simulation::simulated_arena::kSubDeltaTime;
        m_poseEstimator.UpdateWithTime(
            frc::Timer::GetFPGATimestamp() - deltaTime,
            m_gyroSimulation.GetGyroReading(),
            GetModulePositions()
        );
    }

    frc::SwerveDrivePoseEstimator<4>& SwerveDriveSimulation::GetPoseEstimator() {
        return m_poseEstimator;
    }

    //--------------------- Private ---------------------

    /**
     * Simulates the translational friction that the robot has when
     * sliding across the floor. 
     * 
     * Without this function, modules would have no grip on the floor
     * and the robot would slide freely. This prevents that by giving
     * a rough estimation of the floor grip
     */
    void SwerveDriveSimulation::SimulateChassisFrictionForce() {
        using namespace simulation::commonmath; // Shortens simulation::commonmath into just commonmath

        frc::ChassisSpeeds moduleSpeeds = GetRobotRelativeSpeeds();
        frc::ChassisSpeeds differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative{
            moduleSpeeds - GetPhysicsRobotRelativeSpeeds()
        };
                
        frc::Translation2d floorAndModuleSpeedsDiff{
            units::meter_t{ differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vx.value() },
            units::meter_t{ differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vy.value() }
        };

        floorAndModuleSpeedsDiff = floorAndModuleSpeedsDiff.RotateBy(GetPhysicsPose().Rotation());
                
        double FRICTION_FORCE_GAIN = 3.0;
        units::newton_t totalGrippingForce{ 
            m_modules[0].GetGrippingForce(m_moduleGravityForce)
            * m_modules.size()
        };

        units::newton_t finalGrippingForce{units::math::min(
            FRICTION_FORCE_GAIN * totalGrippingForce * floorAndModuleSpeedsDiff.Norm().value(),
            totalGrippingForce
        )};

        b2Vec2 speedsDifferenceFrictionForce{commonmath::PolarToCartesian(
            finalGrippingForce,
            commonmath::GetAngle(floorAndModuleSpeedsDiff).Radians()
        )};

        /* the centripetal friction force during turning */
        frc::ChassisSpeeds moduleSpeedsFieldRelative{ 
            frc::ChassisSpeeds::FromRobotRelativeSpeeds(moduleSpeeds, GetPhysicsPose().Rotation()) 
        };

        frc::Rotation2d dTheta{
            commonmath::GetAngle(commonmath::GetTranslationalSpeeds(moduleSpeedsFieldRelative))
            - commonmath::GetAngle(m_previousModuleSpeeds)
        };

        units::radians_per_second_t orbitalAngularVelocity{
            dTheta.Radians() / constants::simulation::simulated_arena::kSubDeltaTime
        };

        frc::Rotation2d centripetalForceDirection{
            commonmath::GetAngle(m_previousModuleSpeeds) + frc::Rotation2d{ 90_deg }
        };

        b2Vec2 centripetalFrictionForce{
            commonmath::PolarToCartesian(
                units::newton_t{ m_previousModuleSpeeds.Norm().value() * orbitalAngularVelocity.value() * m_config.robotMass.value() },
                centripetalForceDirection.Radians()
            )
        };

        m_previousModuleSpeeds = commonmath::GetTranslationalSpeeds(moduleSpeedsFieldRelative);

        /* apply force to physics engine */
        b2Vec2 totalFrictionForceUnlimited = centripetalFrictionForce + speedsDifferenceFrictionForce;
        b2Vec2 totalFrictionForce{ commonmath::PolarToCartesian(
            units::math::min(totalGrippingForce, units::newton_t{ totalFrictionForceUnlimited.Length() }),
            units::radian_t{ b2Atan2(totalFrictionForceUnlimited.y, totalFrictionForceUnlimited.x) } // direction of total friction force unilimited
        )};
        
        m_body->ApplyForce(totalFrictionForce, m_body->GetWorldCenter(), true);
    }  

    /**
     * Simulates the friction the robot has when rotating
     */
    void SwerveDriveSimulation::SimulateChassisFrictionTorque() {
        const units::radians_per_second_t maxAngularVelocity = GetMaxAngularVelocity();
        
        const double desiredRotationalMotionPercent = units::math::abs(
            GetDesiredSpeeds().omega / maxAngularVelocity
        );
        const double actualRotationalMotionPercent = units::math::abs(
            units::radians_per_second_t{ m_body->GetAngularVelocity() } 
            / maxAngularVelocity
        );
        const units::radians_per_second_t slipAngularVelocity = 
            GetRobotRelativeSpeeds().omega - units::radians_per_second_t{ m_body->GetAngularVelocity() };
    
        const units::newton_meter_t maxGrippingTorque = 
            m_modules[0].GetGrippingForce(m_moduleGravityForce)
            * m_moduleTranslations[0].Norm()
            * m_modules.size();
        
        constexpr double kFrictionTorqueGain = 1;

        // If desired and actual rotation are pretty close to each other it's fine
        // just early return
        if (actualRotationalMotionPercent < 0.01 && desiredRotationalMotionPercent < 0.02) {
            m_body->SetAngularVelocity(0);
            return;
        } 
        
        // Scale torque with slip, but make sure it never exceeds physical grip
        // limit by clamping it by using std::min
        const units::newton_meter_t frictionTorqueMagnitude = units::math::min(
            kFrictionTorqueGain
            * maxGrippingTorque
            * units::math::abs(slipAngularVelocity).value(),
            maxGrippingTorque
        );

        const units::newton_meter_t frictionTorque = units::newton_meter_t{
            units::math::copysign(frictionTorqueMagnitude, slipAngularVelocity)
        };

        // b2Body accepts newton meters for torque
        m_body->ApplyTorque(frictionTorque.value(), true);
    }

    /**
     * Simulates the propelling forces that move the 
     * robot in the physics world.
     * 
     * It obtains the force each module is generating, given its
     * current voltage, power, etc. and applies that force to the body to
     * where the module is located
     */
    void SwerveDriveSimulation::SimulateModulePropellingForces() {
        for (int i = 0; auto& moduleSimulation : m_modules) {
            const frc::Translation2d& moduleTranslation = m_moduleTranslations[i];
            const b2Vec2 moduleWorldPosition = m_body->GetWorldPoint(
                simulation::commonmath::ToBox2dVector2(moduleTranslation)
            );

            const b2Vec2 moduleForce = moduleSimulation.ComputeModuleForce(
                m_body->GetLinearVelocityFromWorldPoint(moduleWorldPosition), // includes ω × r
                GetPhysicsPose().Rotation(),
                m_moduleGravityForce
            );

            m_body->ApplyForce(moduleForce, moduleWorldPosition, true);
            i++;
        }
    }
}