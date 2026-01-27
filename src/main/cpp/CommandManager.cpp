#include "CommandManager.h"



namespace CommandManager {

  frc2::CommandPtr ResetPose(SwerveDrive *swerveDrive) {
    return frc2::InstantCommand([swerveDrive]{
      if (frc::DriverStation::GetAlliance()) {
        if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
          swerveDrive->SetAutoPose(AutoAlignConstants::kBlueHubPose, false);
        else
          swerveDrive->SetAutoPose(AutoAlignConstants::kRedHubPose, false);
      }
    },{swerveDrive}).ToPtr();
  }

  frc2::CommandPtr TurretPowerForward(Turret *turret) {
    return SetShooterRotationPower(turret, 0.2).ToPtr(); 
  }

  frc2::CommandPtr TurretPowerReverse(Turret *turret) {
    return SetShooterRotationPower(turret, -0.2).ToPtr(); 
  }

  frc2::CommandPtr ResetTurretPower(Turret *turret) {
    return SetShooterRotationPower(turret, 0.0).ToPtr(); 
  }

  frc2::CommandPtr AlignToHub(Turret *turret, Articulation *articulation) {
    return frc2::ParallelCommandGroup(
      SetTurretAngle(turret, TurretTarget::AlignHub),
      SetArticulationAngle(articulation, ArticulationTarget::AligntoHub)
    ).ToPtr(); 
  }

  frc2::CommandPtr PointTurretStraight(Turret *turret) {
    return SetTurretAngle(turret, TurretTarget::PointStraight).ToPtr(); 
  }

  frc2::CommandPtr PointTurretLeft(Turret *turret) {
    return SetTurretAngle(turret, TurretTarget::PointFerryLeft).ToPtr();
  }

  frc2::CommandPtr PointTurretRight(Turret *turret) {
    return SetTurretAngle(turret, TurretTarget::PointFerryRight).ToPtr();
  }

  frc2::CommandPtr SlowDownSwerves(SwerveDrive *swerveDrive, DriveState *driveState, frc::PS5Controller *bill) {
    return frc2::InstantCommand([swerveDrive, bill, driveState]{
      *driveState = DriveState::SlowDownSwerves;
      swerveDrive->SetDefaultCommand(Drive(bill, swerveDrive, *driveState));
    },{swerveDrive}).ToPtr();
  }

  frc2::CommandPtr DefaultSwerveSpeed(SwerveDrive *swerveDrive, DriveState *driveState, frc::PS5Controller *bill) {
    return frc2::InstantCommand([swerveDrive, driveState, bill]{
      *driveState = DriveState::HeadingControl;
      swerveDrive->SetDefaultCommand(Drive(bill, swerveDrive, *driveState));
    },{swerveDrive}).ToPtr();
  }

  frc2::CommandPtr IntakePowerForward(Intake *Intake){
    return SetIntakeRollerMotor(Intake, 0.4).ToPtr();
  }

  frc2::CommandPtr IntakePowerReverse(Intake *Intake){
    return SetIntakeRollerMotor(Intake, -0.4).ToPtr();
  }

  frc2::CommandPtr ResetIntakePower(Intake *Intake){
    return SetIntakeRollerMotor(Intake, 0.0).ToPtr();
  }

  frc2::CommandPtr FlywheelPowerForward(Flywheel *flywheel){
    return SetFlywheelMotorPower(flywheel, 0.8).ToPtr();
  }

  frc2::CommandPtr FlywheelPowerReverse(Flywheel *flywheel){
    return SetFlywheelMotorPower(flywheel, -0.8).ToPtr();
  }

  frc2::CommandPtr ResetFlywheelPower(Flywheel *flywheel){
    return SetFlywheelMotorPower(flywheel, 0.0).ToPtr();
  }



};