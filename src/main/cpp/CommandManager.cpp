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
    return SetShooterRotationPower(turret, 0.175).ToPtr(); 
  }

  frc2::CommandPtr TurretPowerReverse(Turret *turret) {
    return SetShooterRotationPower(turret, -0.175).ToPtr(); 
  }

  frc2::CommandPtr ResetTurretPower(Turret *turret) {
    return SetShooterRotationPower(turret, 0.0).ToPtr(); 
  }

  frc2::CommandPtr AlignToHub(Turret *turret, Articulation *articulation) {
    return frc2::ParallelCommandGroup(
      SetTurretAngle(turret, TurretTarget::AlignHub),
      SetArticulationAngle(articulation, ArticulationTarget::AligntoHub)
    ).ToPtr(); 
    return SetTurretAngle(turret, TurretTarget::AlignHub).ToPtr(); 
  }

  frc2::CommandPtr PointTurretStraight(Turret *turret) {
    return SetTurretAngle(turret, TurretTarget::PointStraight).ToPtr(); 
  }

  frc2::CommandPtr PointTurretLeft(Turret *turret) {
    return SetTurretAngle(turret, TurretTarget::Point45Deg).ToPtr();
  }

  frc2::CommandPtr PointTurretRight(Turret *turret) {
    return SetTurretAngle(turret, TurretTarget::Point135Deg).ToPtr();
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
  
  frc2::CommandPtr ClimbAlignSwerves(SwerveDrive *swerveDrive, DriveState *driveState, frc::PS5Controller *bill) {
    return frc2::InstantCommand([swerveDrive, driveState, bill]{
      if(*driveState == DriveState::ClimbAlign){
        *driveState = DriveState::HeadingControl;
      } else {
        *driveState = DriveState::ClimbAlign; 
      }
      swerveDrive->SetDefaultCommand(Drive(bill, swerveDrive, *driveState));
    },{swerveDrive}).ToPtr();
  }

  SetIntakeRollerMotor IntakePowerForward(Intake *intake){
    return SetIntakeRollerMotor(intake, IntakeConstants::kRollerIntakePower);
  }

  frc2::CommandPtr IntakePowerReverse(Intake *intake){
    return SetIntakeRollerMotor(intake, IntakeConstants::kRollerReverseIntakePower).ToPtr();
  }

  frc2::CommandPtr ResetIntakePower(Intake *intake){
    return SetIntakeRollerMotor(intake, 0.0).ToPtr();
  }

  frc2::CommandPtr IntakeForwardManual(Intake *intake){
    return SetIntakeManual(intake, IntakeState::ExtendIntake, IntakeConstants::kRollerIntakePower).ToPtr();
  }

  frc2::CommandPtr IntakeBackwardManual(Intake *intake){
    return SetIntakeManual(intake, IntakeState::RetractIntake, 0.0).ToPtr();
  }

  frc2::CommandPtr FlywheelPowerForward(Flywheel *flywheel){
    return SetFlywheelMotorPower(flywheel, -0.15).ToPtr();
  }

  frc2::CommandPtr FlywheelPowerReverse(Flywheel *flywheel){
    return SetFlywheelMotorPower(flywheel, 0.3).ToPtr();
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

  frc2::CommandPtr RunArticulationForward(Articulation *articulation) {
    return frc2::InstantCommand([articulation]() {
      articulation->SetArticulationMotorPower(0.1); 
    }).ToPtr();
  }
    

  frc2::CommandPtr RunArticulationReverse(Articulation *articulation) {
    return frc2::InstantCommand([articulation]() {
      articulation->SetArticulationMotorPower(-0.1); 
    }).ToPtr();
  }
  frc2::CommandPtr ResetArticulationPower(Articulation *articulation) {
    return frc2::InstantCommand([articulation]() {
      articulation->SetArticulationMotorPower(0.0); 
    }).ToPtr();
  }

  frc2::CommandPtr Articulation20Degrees(Articulation *articulation) {
    return SetArticulationAngle(articulation, ArticulationTarget::Degrees20).ToPtr(); 
  }
    frc2::CommandPtr Articulation25Degrees(Articulation *articulation) {
    return SetArticulationAngle(articulation, ArticulationTarget::Degrees25).ToPtr(); 
  }
  frc2::CommandPtr Articulation30Degrees(Articulation *articulation) {
    return SetArticulationAngle(articulation, ArticulationTarget::Degrees30).ToPtr(); 
  }
  frc2::CommandPtr Articulation40Degrees(Articulation *articulation) {
    return SetArticulationAngle(articulation, ArticulationTarget::Degrees40).ToPtr(); 
  }
 
  frc2::CommandPtr SetArticulationToFixedState(Articulation* articulation) {
    return SetArticulationAngle(articulation, ArticulationTarget::FixedAlign).ToPtr();
  }

  frc2::CommandPtr SetTurretAndArticulationManual(Articulation *articulation, Turret *turret) {
    return frc2::SequentialCommandGroup(
      SetArticulationAngle(articulation, ArticulationTarget::FixedAlign),
      SetTurretAngle(turret, TurretTarget::PointStraight)
    ).ToPtr();
  }

  frc2::CommandPtr AutoShoot(Flywheel *flywheel, Kicker *kicker, Loader *loader, Articulation *articulation) {
    return frc2::SequentialCommandGroup(
      // SetArticulationAngle(articulation, ArticulationTarget::Degrees20),
      frc2::ParallelCommandGroup(
        frc2::SequentialCommandGroup(
          SetFlywheelMotorPower(flywheel, FlywheelConstants::kAutoStaticFlywheelPower),
          frc2::WaitCommand(FlywheelConstants::kFlywheelRunDelay)
        ),
        SetLoaderPower(loader, -LoaderConstants::kLoadFuelPower)
      ),
      frc2::ParallelCommandGroup(
        SetLoaderPower(loader, LoaderConstants::kLoadFuelPower),
        SetShooterKickerPower(kicker, KickerConstants::kKickerMotorPower)
      )
    ).ToPtr();
  }

  frc2::CommandPtr AutoShootEnd(Flywheel *flywheel, Kicker *kicker, Loader *loader, Articulation *articulation) {
    return frc2::SequentialCommandGroup(
          SetFlywheelMotorPower(flywheel, 0.0), 
          SetLoaderPower(loader, 0.0),
          SetShooterKickerPower(kicker, 0.0),
          SetArticulationAngle(articulation, ArticulationTarget::Degrees0)
        ).ToPtr();
  }
  
  frc2::CommandPtr RunKickerForward(Kicker *kicker) {
    return SetShooterKickerPower(kicker, 0.4).ToPtr();
  }

  frc2::CommandPtr RunKickerBackward(Kicker *kicker) {
    return SetShooterKickerPower(kicker, -0.4).ToPtr();
  }

  frc2::CommandPtr ResetKicker(Kicker *kicker) {
    return SetShooterKickerPower(kicker, 0.0).ToPtr();
  }

  frc2::CommandPtr RunLoaderForward(Loader *loader) {
    return SetLoaderPower(loader, 0.4).ToPtr();
  }

  frc2::CommandPtr RunLoaderBackward(Loader *loader) {
    return SetLoaderPower(loader, -0.4).ToPtr();
  }

  frc2::CommandPtr ResetLoader(Loader *loader) {
    return SetLoaderPower(loader, 0.0).ToPtr();
  }

  frc2::SequentialCommandGroup Shoot(Turret *turret, Flywheel *flywheel, Kicker *kicker, Loader *loader, Articulation *articulation) {
    return frc2::SequentialCommandGroup(
      // SetTurretAngle(turret, TurretTarget::AlignHub),
      // SetArticulationAngle(articulation, ArticulationTarget::Degrees20),
      frc2::ParallelCommandGroup(
        frc2::SequentialCommandGroup(
          SetFlywheelMotorPower(flywheel, FlywheelConstants::kFlywheelShootPower),
          frc2::WaitCommand(FlywheelConstants::kFlywheelRunDelay)
        ),
        SetLoaderPower(loader, -LoaderConstants::kLoadFuelPower)
      ),
      frc2::ParallelCommandGroup(
        SetLoaderPower(loader, LoaderConstants::kLoadFuelPower),
        SetShooterKickerPower(kicker, KickerConstants::kKickerMotorPower)
      )
    );
  }
    
  frc2::SequentialCommandGroup AdvancedShoot(Turret *turret, Flywheel *flywheel, Kicker *kicker, Loader *loader, Articulation *articulation) {
    return frc2::SequentialCommandGroup(
      // SetTurretAngle(turret, TurretTarget::AlignHub),
      // SetArticulationAngle(articulation, ArticulationTarget::AligntoHub),
      frc2::ParallelCommandGroup(
        frc2::SequentialCommandGroup(
          frc2::InstantCommand([flywheel]() {
            flywheel->SetFlywheelMotorPower(); 
          }),
          frc2::WaitCommand(FlywheelConstants::kFlywheelRunDelay)
        ),
        SetLoaderPower(loader, -LoaderConstants::kLoadFuelPower)
      ),
      frc2::ParallelCommandGroup(
        SetLoaderPower(loader, LoaderConstants::kLoadFuelPower),
        SetShooterKickerPower(kicker, KickerConstants::kKickerMotorPower)
      )
    );
  }
    

  frc2::CommandPtr ShootEnd(Flywheel *flywheel, Kicker *kicker, Loader *loader, Articulation *articulation) {
    return frc2::SequentialCommandGroup(
      SetFlywheelMotorPower(flywheel, 0.0), 
      SetLoaderPower(loader, 0.0),
      // SetArticulationAngle(articulation, ArticulationTarget::Stationary),
      SetShooterKickerPower(kicker, 0.0)
    ).ToPtr();
  }

  frc2::CommandPtr NomNomMode(Turret *turret, Flywheel *flywheel, Kicker *kicker, Loader *loader, Intake *intake, 
                                Articulation *articulation, bool *nomNomMode) {
    if (*nomNomMode == false) {
      *nomNomMode = true; 
      return NomNomModeOn(turret, flywheel, kicker, loader, intake, articulation); 
    } else {
      *nomNomMode = false;
      return NomNomModeOff(flywheel, kicker, loader, intake);  
    }
  }

  frc2::CommandPtr NomNomModeOn(Turret *turret, Flywheel *flywheel, Kicker *kicker, Loader *loader, Intake *intake, Articulation *articulation) {    
    return frc2::ParallelCommandGroup(
      Shoot(turret, flywheel, kicker, loader, articulation), 
      SetIntakeAngleAndPower(intake, IntakeConstants::kRollerIntakePower)
    ).ToPtr();
  }

  frc2::CommandPtr NomNomModeOff(Flywheel *flywheel, Kicker *kicker, Loader *loader, Intake *intake) {
    return frc2::SequentialCommandGroup(
      SetFlywheelMotorPower(flywheel, 0.0), 
      SetLoaderPower(loader, 0.0),
      SetShooterKickerPower(kicker, 0.0),
      SetIntakeManual(intake, IntakeState::RetractIntake, 0.0)
    ).ToPtr();
  }

  frc2::CommandPtr AddInterpolatorPoint(InterpolatorData *interpolatorData, SwerveDrive *swerveDrive, 
                                        Flywheel *flywheel, Articulation *articulation) {
    return frc2::InstantCommand([interpolatorData, swerveDrive, flywheel, articulation]() { // saves the point 
      frc::Pose2d robotPose = swerveDrive->GetEstimatedPose(); 
      interpolatorData->AddPoint(robotPose, TurretAlign::GetDistanceToTarget(robotPose), 
                                 articulation->GetArticulationAngle(), flywheel->GetFlywheelTargetPower());
    }, {flywheel, swerveDrive, articulation}).ToPtr();  
  }
  
  frc2::CommandPtr SaveAllInterpolationPoints(InterpolatorData *interpolatorData) {
    return frc2::InstantCommand([interpolatorData]() {
      interpolatorData->SaveToFile(); 
    }).ToPtr(); 
  }

  frc2::CommandPtr RemoveLastInterpolationPoint(InterpolatorData *interpolatorData) {
    return frc2::InstantCommand([interpolatorData]() {
      interpolatorData->RemoveLastPoint(); 
    }).ToPtr(); 
  }

  frc2::CommandPtr GoToClosestPose(SwerveDrive *swerveDrive, pathplanner::PathConstraints constraints) {
    return frc2::cmd::RunOnce([swerveDrive, constraints](){
      frc2::CommandScheduler::GetInstance().Schedule(AutoAlign::AutoAlignToPose(swerveDrive, constraints));
    });
  }

  frc2::CommandPtr EmptyCommand() {
    return frc2::InstantCommand([](){}).ToPtr();
  }

  // frc2::CommandPtr ExtendOrRetractIntake(Intake* intake) {

  // }
  //   frc2::Trigger extendOrRetractIntake([this](){ return m_bill.GetL2Button(); });
  // extendOrRetractIntake.OnTrue(SetIntakeAngleAndPower(m_intake, 0.8).ToPtr()); // 0.8 when set, nothign when retract
  // extendOrRetractIntake.OnFalse(frc2::InstantCommand([](){}, {m_intake}).ToPtr()); 

};