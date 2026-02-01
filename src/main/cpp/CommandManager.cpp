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

  frc2::CommandPtr IntakePowerForward(Intake *intake){
    return SetIntakeRollerMotor(intake, IntakeConstants::kRollerIntakePower).ToPtr();
  }

  frc2::CommandPtr IntakePowerReverse(Intake *intake){
    return SetIntakeRollerMotor(intake, IntakeConstants::kRollerReverseIntakePower).ToPtr();
  }

  frc2::CommandPtr ResetIntakePower(Intake *intake){
    return SetIntakeRollerMotor(intake, 0.0).ToPtr();
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

  frc2::CommandPtr IncrementFlywheelPower(Flywheel *flywheel) {
    return frc2::InstantCommand([flywheel](){
      flywheel->SetFlyWheelTargetPower(flywheel->GetFlywheelTargetPower() + FlywheelConstants::kFlyWheelPowerIncrement); 
    }, {flywheel}).ToPtr();
  } 

  frc2::CommandPtr DecrementFlywheelPower(Flywheel *flywheel) {
    return frc2::InstantCommand([flywheel](){
      flywheel->SetFlyWheelTargetPower(flywheel->GetFlywheelTargetPower() + FlywheelConstants::kFlyWheelPowerDecrement); 
    }, {flywheel}).ToPtr();
  } 

  frc2::CommandPtr ArticulationAngleUp(Articulation *articulation) {
      return frc2::InstantCommand([articulation]() {
        articulation->IncrementManualAngle(ArticulationConstants::kArticulationAngleIncrement);
      }, {articulation}).ToPtr();
  }

  frc2::CommandPtr ArticulationAngleDown(Articulation *articulation) {
      return frc2::InstantCommand([articulation]() {
        articulation->IncrementManualAngle(ArticulationConstants::kArticulationAngleDecrement);
      }, {articulation}).ToPtr();
  }

  frc2::CommandPtr RunFlywheelAndLoader(Loader *loader, Flywheel *flywheel) {
    return frc2::ParallelCommandGroup(
      frc2::InstantCommand([flywheel]() {
        flywheel->SetFlyWheelPowerTest(); 
      }, {flywheel}),
      frc2::SequentialCommandGroup(
        frc2::WaitCommand(FlywheelConstants::kFlywheelRunDelay),
        frc2::InstantCommand([loader]() {
          loader->SetShooterLoaderMotorPower(LoaderConstants::kLoaderMotorPower); 
        }, {loader})
      )
    ).ToPtr(); 
    
  }

  frc2::CommandPtr SetArticulationManual(Articulation *articulation) {
    return SetArticulationAngle(articulation, ArticulationTarget::Manual).ToPtr(); 
  }

  // frc2::CommandPtr ExtendOrRetractIntake(Intake* intake) {

  // }
  //   frc2::Trigger extendOrRetractIntake([this](){ return m_bill.GetL2Button(); });
  // extendOrRetractIntake.OnTrue(SetIntakeAngleAndPower(m_intake, 0.8).ToPtr()); // 0.8 when set, nothign when retract
  // extendOrRetractIntake.OnFalse(frc2::InstantCommand([](){}, {m_intake}).ToPtr()); 



};