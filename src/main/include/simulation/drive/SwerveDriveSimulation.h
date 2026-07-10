#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <units/mass.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/torque.h>
#include <units/current.h>

#include "constants/swerve/SwerveDrive.h"

#include "simulation/CommonMath.h" 
#include "simulation/drive/SwerveModuleSimulation.h"
#include "simulation/drive/GyroSimulation.h"
#include "simulation/box2d/box2d.h"



namespace simulation::drive {
    /**
     * A class for simulating a swerve drive on the field.
     * The reason it's customizable and you have to pass in
     * constants directly (like module translations) is for if
     * you'd like other unique drive trains on the field
     * alongside yours.
     */
    class SwerveDriveSimulation {
    public:
        struct Config {
            units::kilogram_t robotMass;
            units::meter_t bumperLengthX;   // field-forward dimension
            units::meter_t bumperWidthY;    // field-sideways dimension
            units::meter_t trackLengthX;
            units::meter_t trackWidthY;

            std::array<frc::Translation2d, 4> moduleTranslations;
            std::array<SwerveModuleSimulation::Config, 4> moduleConfigs;

            GyroSimulation gyroSimulation;
        };

        SwerveDriveSimulation(Config config, frc::Pose2d initialPose);
        ~SwerveDriveSimulation();

        // Movement
        void Stop();
        frc::ChassisSpeeds GetRobotRelativeSpeeds() const; // Get chassis speeds
        frc::ChassisSpeeds GetPhysicsRobotRelativeSpeeds() const;
        frc::ChassisSpeeds GetDesiredSpeeds() const;
        void RunSwerveStates(std::array<frc::SwerveModuleState, 4> states);
        void RunChassisSpeeds(frc::ChassisSpeeds speeds, frc::Translation2d center, bool fieldRelative);
        std::array<SwerveModuleSimulation, 4>& GetModules();

        // Positioning
        wpi::array<frc::SwerveModulePosition, 4> GetModulePositions() const;
        frc::Pose2d GetOdometryEstimatedPose() const;
        frc::Pose2d GetPhysicsPose() const;
        void SetPhysicsPose(frc::Pose2d pose);
        void ResetOdometry(frc::Pose2d pose);

        // Maximums
        units::radians_per_second_t GetMaxAngularVelocity() const;
        units::radians_per_second_squared_t GetMaxAngularAcceleration(units::ampere_t statorCurrentLimit) const;
        
        // Not const because we'd like for others to have freedom
        // to change and modify it
        GyroSimulation& GetGyroSimulation();
        frc::SwerveDriveKinematics<4>& GetSwerveDriveKinematics();
        frc::SwerveDrivePoseEstimator<4>& GetPoseEstimator();

        void SimulationSubTick(int subtickNum);

    private:
        std::array<SwerveModuleSimulation, 4> CreateModules(Config config);

        void SimulateChassisFrictionForce();
        void SimulateChassisFrictionTorque();
        void SimulateModulePropellingForces();
        
        Config m_config;
        wpi::array<frc::Translation2d, 4> m_moduleTranslations;

        b2Body* m_body;        
        std::array<SwerveModuleSimulation, 4> m_modules;
        frc::SwerveDriveKinematics<4> m_kinematics;
        GyroSimulation m_gyroSimulation;
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator;
        
        units::newton_t m_moduleGravityForce;
        frc::Translation2d m_previousModuleSpeeds;
    };
}