#include "subsystems/swerve/SimDrive.h"

namespace subsystems::swerve {
    /**
     * Creates the main SimDrive used for simulation. Not to be confused with
     * SwerveDriveSimulation, which can have multiple instances at once, and
     * can be controlled by any controller, person, or computer.
     */
    SimDrive::SimDrive(vision::PoseEstimatorHelper* helper) 
    : m_simulatedDrive(
        simulation::drive::SwerveDriveSimulation::Config{
            .robotMass = 45_kg,
            .bumperLengthX = constants::swerve::swerve_drive::kTrackwidth,
            .bumperWidthY = constants::swerve::swerve_drive::kTrackwidth,
            .trackLengthX = constants::swerve::swerve_drive::kTrackwidth,
            .trackWidthY = constants::swerve::swerve_drive::kTrackwidth,
            
            .moduleTranslations = {
                frc::Translation2d{+constants::swerve::swerve_drive::kXDistanceFromCenter, +constants::swerve::swerve_drive::kYDistanceFromCenter},
                frc::Translation2d{+constants::swerve::swerve_drive::kXDistanceFromCenter, -constants::swerve::swerve_drive::kYDistanceFromCenter},
                frc::Translation2d{-constants::swerve::swerve_drive::kXDistanceFromCenter, +constants::swerve::swerve_drive::kYDistanceFromCenter},
                frc::Translation2d{-constants::swerve::swerve_drive::kXDistanceFromCenter, -constants::swerve::swerve_drive::kYDistanceFromCenter}
            },

            //! oh my god this is awful
            .moduleConfigs = {
                simulation::drive::SwerveModuleSimulation::Config{
                    .driveMotorModel = frc::DCMotor::KrakenX60FOC(1),
                    .driveGearRatio = 6.75,
                    .wheelRadius = 2_in,
                    .driveFrictionVoltage = 0.1_V,
                    .driveCurrentLimit = 60_A,

                    .angleMotorModel = frc::DCMotor::KrakenX60FOC(1),
                    .angleGearRatio = 21.4286,                       
                    .angleFrictionVoltage = 0.25_V,
                    .driveMOI = 0.025_kg_sq_m,
                    .angleMOI = 0.04_kg_sq_m,
                    .angleKp = 8.0,
                    .angleKd = 0.8,

                    .wheelCOF = 1.2
                },
                 simulation::drive::SwerveModuleSimulation::Config{
                    .driveMotorModel = frc::DCMotor::KrakenX60FOC(1),
                    .driveGearRatio = 6.75,
                    .wheelRadius = 2_in,
                    .driveFrictionVoltage = 0.1_V,
                    .driveCurrentLimit = 60_A,

                    .angleMotorModel = frc::DCMotor::KrakenX60FOC(1),
                    .angleGearRatio = 21.4286,                       
                    .angleFrictionVoltage = 0.25_V,
                    .driveMOI = 0.025_kg_sq_m,
                    .angleMOI = 0.04_kg_sq_m,
                    .angleKp = 8.0,
                    .angleKd = 0.8,

                    .wheelCOF = 1.2
                }, 
                 simulation::drive::SwerveModuleSimulation::Config{
                    .driveMotorModel = frc::DCMotor::KrakenX60FOC(1),
                    .driveGearRatio = 6.75,
                    .wheelRadius = 2_in,
                    .driveFrictionVoltage = 0.1_V,
                    .driveCurrentLimit = 60_A,

                    .angleMotorModel = frc::DCMotor::KrakenX60FOC(1),
                    .angleGearRatio = 21.4286,                       
                    .angleFrictionVoltage = 0.25_V,
                    .driveMOI = 0.025_kg_sq_m,
                    .angleMOI = 0.04_kg_sq_m,
                    .angleKp = 8.0,
                    .angleKd = 0.8,

                    .wheelCOF = 1.2
                }, 
                 simulation::drive::SwerveModuleSimulation::Config{
                    .driveMotorModel = frc::DCMotor::KrakenX60FOC(1),
                    .driveGearRatio = 6.75,
                    .wheelRadius = 2_in,
                    .driveFrictionVoltage = 0.1_V,
                    .driveCurrentLimit = 60_A,

                    .angleMotorModel = frc::DCMotor::KrakenX60FOC(1),
                    .angleGearRatio = 21.4286,                       
                    .angleFrictionVoltage = 0.25_V,
                    .driveMOI = 0.025_kg_sq_m,
                    .angleMOI = 0.04_kg_sq_m,
                    .angleKp = 8.0,
                    .angleKd = 0.8,

                    .wheelCOF = 1.2
                }
            },
            .gyroSimulation = simulation::drive::GyroSimulation(0.5_deg, 0.02)
        },
        frc::Pose2d(constants::swerve::swerve_drive::kRedResetPose)
    )

    , m_poseHelper(helper)
    {   
        m_poseHelper->SetPoseEstimator(&m_simulatedDrive.GetPoseEstimator());

        ConfigurePathPlannerAutoBuilder();
    };


    // Motion methods

    void SimDrive::Stop() {
        m_simulatedDrive.Stop();
    }

    void SimDrive::Operate(
        units::meters_per_second_t xSpeed,
        units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
        bool fieldRelative,
        frc::Translation2d centerOfRotation
    ) {
        m_simulatedDrive.RunChassisSpeeds(
            frc::ChassisSpeeds{ xSpeed, ySpeed, rot },
            frc::Translation2d{},
            fieldRelative
        );
    }

    void SimDrive::DriveRobotRelative(frc::ChassisSpeeds speeds) {
        if (speeds.omega > units::radians_per_second_t(0.0)){
            speeds.omega += units::radians_per_second_t(constants::swerve::swerve_drive::kSRot);
        } else if (speeds.omega < units::radians_per_second_t(0.0)) {
            speeds.omega -= units::radians_per_second_t(constants::swerve::swerve_drive::kSRot);
        }

        frc::SwerveDriveKinematics swerveKinematics = m_simulatedDrive.GetSwerveDriveKinematics();
        wpi::array<frc::SwerveModuleState, 4U> states = swerveKinematics.ToSwerveModuleStates(speeds);
        swerveKinematics.DesaturateWheelSpeeds(&states, constants::automatic::kMaxAutoModuleSpeed);

        SetModuleStates(states);
    }


    void SimDrive::SetModuleStates(std::array<frc::SwerveModuleState, 4> states) {
        m_simulatedDrive.RunSwerveStates(states);
    }

    // Pose/odometry methods
    void SimDrive::UpdateEstimator() {
        wpi::array<frc::SwerveModulePosition, 4U> modulePositions = m_simulatedDrive.GetModulePositions();
        
        units::second_t robotTime = frc::Timer::GetTimestamp();
        frc::Rotation2d yaw{ GetNormalizedYaw() };
        m_poseHelper->UpdatePoseEstimator(robotTime + m_timestampOffset, modulePositions, yaw);
        LogModuleStates(modulePositions);
    }

    units::degree_t SimDrive::GetNormalizedYaw() {
        return m_simulatedDrive.GetGyroSimulation().GetGyroReading().Degrees();
    }
    
    void SimDrive::SetBrakeMode(constants::BrakeMode mode) {
        // Simulated modules don't use brake mode
    }

    void SimDrive::SetPose(frc::Pose2d pose, bool justRotation) {
        UpdateEstimator();
        
        frc::Pose2d currentPose = GetEstimatedPose();
        frc::Pose2d finalPose = justRotation ? frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()} : pose;

        m_simulatedDrive.SetPhysicsPose(finalPose);
        m_simulatedDrive.ResetOdometry(finalPose);
    }

    void SimDrive::SetAutoPose(frc::Pose2d pose, bool justRotation) {
        UpdateEstimator();

        auto modulePositions = m_simulatedDrive.GetModulePositions();
        if (justRotation) {
            frc::Pose2d currentPose = GetEstimatedPose();
            m_poseHelper->ResetPose(GetNormalizedYaw(), modulePositions, frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()});
        }
        else {
            m_poseHelper->ResetPose(GetNormalizedYaw(), modulePositions, pose);
        }
        //sets pose to current pose
        frc::SmartDashboard::PutNumber("Auto starting pose x", pose.X().value());
        frc::SmartDashboard::PutNumber("Auto starting pose y", pose.Y().value());
        frc::SmartDashboard::PutNumber("Auto starting pose rot", pose.Rotation().Degrees().value());
    }

    frc::Pose2d SimDrive::GetEstimatedPose() {
        return m_simulatedDrive.GetOdometryEstimatedPose();
    }
    
    // Sensor methods

    frc::ChassisSpeeds SimDrive::GetRobotRelativeSpeeds() {
        return m_simulatedDrive.GetRobotRelativeSpeeds();
    }


    // Configuration methods
    
    void SimDrive::SetSlowMode(bool slow) {
        m_slowMode = slow;
    }    

    void SimDrive::SetTimestampOffset(units::second_t offset) {
        m_timestampOffset = offset;
    }
        
    void SimDrive::LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions) {
        auto modules = m_simulatedDrive.GetModules();
        
        double AdvantageScopeMeasuredStates[] = {
            modulePositions[0].angle.Degrees().value(), modules[0].GetDriveSpeed().value(),
            modulePositions[1].angle.Degrees().value(), modules[1].GetDriveSpeed().value(),
            modulePositions[2].angle.Degrees().value(), modules[2].GetDriveSpeed().value(),
            modulePositions[3].angle.Degrees().value(), modules[3].GetDriveSpeed().value()
        };

        frc::SmartDashboard::PutNumberArray("AdvantageScope Measured States", AdvantageScopeMeasuredStates);
    }


    // Other

    void SimDrive::SimulationPeriodic() { 
        for (int i = 0; i < constants::simulation::simulated_arena::kSubTicks; i++) {
            m_simulatedDrive.SimulationSubTick(i);
        }

        m_field.SetRobotPose(m_simulatedDrive.GetPhysicsPose());
        m_field.GetObject("odometry")->SetPose(GetEstimatedPose());
        frc::SmartDashboard::PutData("Estimated sim pose", &m_field);

        UpdateEstimator();
    }
}