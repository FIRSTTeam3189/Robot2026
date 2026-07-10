#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Rotation2d.h>

#include <units/force.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/impedance.h>
#include <units/math.h>
#include <units/torque.h>

#include "simulation/box2d/box2d.h"

namespace simulation::drive {
    class SwerveModuleSimulation {
    public:
        struct Config {
            // Drive motor
            frc::DCMotor driveMotorModel;
            double driveGearRatio;
            units::meter_t wheelRadius;
            units::volt_t driveFrictionVoltage;
            units::ampere_t driveCurrentLimit;

            // Angle motor
            frc::DCMotor angleMotorModel;
            double angleGearRatio;
            units::volt_t angleFrictionVoltage;
            units::kilogram_square_meter_t driveMOI;
            units::kilogram_square_meter_t angleMOI;
            double angleKp;
            double angleKd;

            // Wheel
            double wheelCOF;
        };
    
        explicit SwerveModuleSimulation(const Config& config);

        // Control

        void Stop();
        b2Vec2 ComputeModuleForce(b2Vec2 bodyLinearVelocity, frc::Rotation2d bodyRotation, units::newton_t gravityForce);
        void RunSwerveState(const frc::SwerveModuleState& state);
        void SimulationSubTick();

        // Getters

        frc::SwerveModuleState GetState() const;
        frc::SwerveModuleState GetEquilibriumState() const;
        frc::SwerveModulePosition GetPosition() const;
        const Config& GetConfig() const;
        units::meters_per_second_t GetDriveSpeed() const;
        units::newton_t GetGrippingForce(units::newton_t gravityForce) const;
        frc::Rotation2d GetAngleMotorAbsoluteFacing() const;
        units::meters_per_second_t GetMaxGroundSpeed() const;
        units::newton_t GetMaxPropellingForce(
            units::kilogram_t robotMass,
            size_t moduleCount,
            units::ampere_t statorCurrentLimit,
            units::newton_t gravityForcePerModule
        ) const;

    private:
        units::ampere_t CalculateStatorCurrent(
            units::volt_t appliedVoltage,
            units::radians_per_second_t motorVelocity,
            units::ohm_t windingResistance,
            frc::DCMotor::radians_per_second_per_volt_t Kv
        ) const;

        units::radians_per_second_t CalculateDriveMechanismVelocity(
            units::ampere_t current, 
            units::volt_t voltage
        ) const;

        units::ampere_t CalculateDriveCurrent(units::newton_meter_t torque) const;

        Config  m_config;

        units::volt_t m_driveAppliedVoltage{ 0 };
        units::radians_per_second_t m_driveMotorVelocity{ 0 };
        units::radian_t m_driveMotorPosition{ 0 };

        units::ohm_t m_driveMotorWindingResistance{ 0 };
        frc::DCMotor::radians_per_second_per_volt_t m_driveMotorKv{ 0 }; // rad/s/volt
        
        units::volt_t m_angleAppliedVoltage{ 0 };
        units::radians_per_second_t m_angleMotorVelocity{ 0 };
        frc::Rotation2d m_angleRotation;
        frc::Rotation2d m_targetAngle;

        units::ohm_t m_angleMotorWindingResistance{ 0 };
        frc::DCMotor::radians_per_second_per_volt_t m_angleMotorKv{ 0 }; // rad/s/volt
    };
}