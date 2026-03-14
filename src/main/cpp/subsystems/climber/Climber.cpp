// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/climber/Climber.h"


namespace subsystems::climber
{
    Climber::Climber() :
        m_climberMotor(constants::climber::kClimberMotorID, "Swerve"),
        m_climberConfig() 
    {
        ConfigClimberMotor();
    }

    void Climber::ConfigClimberMotor() {
        m_climberMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

        m_climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants::climber::kTopSoftLimitEnabled;
        m_climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants::climber::kBottomSoftLimitEnabled;
        m_climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants::climber::kTopSoftLimit;
        m_climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants::climber::kBottomSoftLimit;

        m_climberConfig.CurrentLimits.SupplyCurrentLowerLimit = constants::climber::kExtensionContinuousCurrentLimit;
        m_climberConfig.CurrentLimits.SupplyCurrentLimit = constants::climber::kExtensionPeakCurrentLimit;
        m_climberConfig.CurrentLimits.SupplyCurrentLowerTime = constants::climber::kExtensionPeakCurrentDuration;
        m_climberConfig.CurrentLimits.SupplyCurrentLimitEnable = constants::climber::kExtensionEnableCurrentLimit;

        m_climberConfig.MotorOutput.Inverted = constants::climber::kExtensionMotorInverted;
        m_climberConfig.MotorOutput.NeutralMode = constants::climber::kExtensionNeutralMode;

        m_climberMotor.GetConfigurator().Apply(m_climberConfig);
    }

    void Climber::SetClimberPower(double power) {
        m_climberMotor.Set(power);
    }

    // This method will be called once per scheduler run
    void Climber::Periodic() {
        frc::SmartDashboard::PutNumber("Climber turns", m_climberMotor.GetPosition().GetValue().value()); // for soft limits
    }

};

