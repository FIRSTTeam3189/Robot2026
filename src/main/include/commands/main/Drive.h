#pragma once

#include <frc/Preferences.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/Joystick.h>
#include <frc/PS5Controller.h>
#include "subsystems/swerve/Drive.h"

#include "constants/Controller.h"
#include "constants/swerve/SwerveDrive.h"
#include "util/align/Turret.h"

namespace commands {
      namespace main {
            class Drive
                : public frc2::CommandHelper<frc2::Command, Drive> {
             public:
              Drive(frc::PS5Controller *joystick, subsystems::swerve::Drive *swerveDrive, subsystems::swerve::State driveState, 
                    units::degree_t arbitraryAngle = 0.0_deg);
              units::angular_velocity::radians_per_second_t GetDesiredRotationalVelocity();
              units::meters_per_second_t LimitTranslationalBranchAlignSpeeds(units::meters_per_second_t currentSpeed, units::meter_t targetDistance, units::meter_t currentDistance);
              units::angular_velocity::radians_per_second_t LimitRotationalBranchAlignSpeed(units::radians_per_second_t currentSpeed, units::degree_t targetAngle, units::degree_t currentAngle);
            
              // For PID tuning
              void UpdatePreferences();
            
              void Initialize() override;
            
              void Execute() override;
            
              void End(bool interrupted) override;
            
              bool IsFinished() override;
            
            
            
             private:
              frc::PS5Controller *m_bill;
              subsystems::swerve::Drive *m_swerveDrive;
              frc::PIDController m_rotationPIDController;
              subsystems::swerve::State m_driveState;
              units::degree_t m_arbitraryAngle;
              frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{2 / 1_s};
              frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{2 / 1_s};
              frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};
              double m_goalAngle;
              double m_lastAngle;
              std::optional<frc::DriverStation::Alliance> m_allianceSide;
              std::string m_rotationPKey;
              std::string m_rotationIKey;
              std::string m_rotationDKey;
            };

      };
};
