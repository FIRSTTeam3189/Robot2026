#pragma once



#include "RobotContainer.h"





namespace CommandManager {

    frc2::CommandPtr ResetPose(SwerveDrive *swerveDrive);

    frc2::CommandPtr TurretPowerForward(Turret *turret);
    frc2::CommandPtr TurretPowerReverse(Turret *turret);
    frc2::CommandPtr ResetTurretPower(Turret *turret);

    frc2::CommandPtr AlignToHub(Turret *turret, Articulation *articulation);

    frc2::CommandPtr PointTurretStraight(Turret *turret);
    frc2::CommandPtr PointTurretLeft(Turret *turret);
    frc2::CommandPtr PointTurretRight(Turret *turret);

    frc2::CommandPtr SlowDownSwerves(SwerveDrive *swerveDrive, DriveState *driveState, frc::PS5Controller *bill);
    frc2::CommandPtr DefaultSwerveSpeed(SwerveDrive *swerveDrive, DriveState *driveState, frc::PS5Controller *bill);

    frc2::CommandPtr IntakePowerForward(Intake *Intake);
    frc2::CommandPtr IntakePowerReverse(Intake *Intake);
    frc2::CommandPtr ResetIntakePower(Intake *Intake);

    frc2::CommandPtr FlywheelPowerForward(Flywheel *flywheel);
    frc2::CommandPtr FlywheelPowerReverse(Flywheel *flywheel);
    frc2::CommandPtr ResetFlywheelPower(Flywheel *flywheel);
    frc2::CommandPtr IncrementFlywheelPower(Flywheel *flywheel); 
    frc2::CommandPtr DecrementFlywheelPower(Flywheel *flywheel);

    frc2::CommandPtr ArticulationAngleUp(Articulation *articulation);
    frc2::CommandPtr ArticulationAngleDown(Articulation *articulation);

    frc2::CommandPtr RunFlywheelAndLoader(Loader *loader, Flywheel *flywheel); 

};