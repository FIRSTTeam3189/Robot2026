#include "actions/Swerve.h"


namespace actions::swerve {
  frc2::InstantCommand SlowDownSwerves(subsystems::swerve::Drive *swerveDrive, subsystems::swerve::State *driveState, 
                                   frc::PS5Controller *bill) {
      return frc2::InstantCommand([swerveDrive, bill, driveState]{
        *driveState = subsystems::swerve::State::SlowDownSwerves;
        swerveDrive->SetDefaultCommand(commands::main::Drive(bill, swerveDrive, *driveState));
      },{swerveDrive});
  }

  frc2::InstantCommand DefaultSwerveSpeed(subsystems::swerve::Drive *swerveDrive, subsystems::swerve::State *driveState, 
                                      frc::PS5Controller *bill) {
    return frc2::InstantCommand([swerveDrive, driveState, bill]{
        *driveState = subsystems::swerve::State::HeadingControl;
        swerveDrive->SetDefaultCommand(commands::main::Drive(bill, swerveDrive, *driveState));
    },{swerveDrive});
  }

  frc2::CommandPtr ClimbAlignSwerves(subsystems::swerve::Drive *swerveDrive, subsystems::swerve::State *driveState, 
                                     frc::PS5Controller *bill) {
    return frc2::InstantCommand([swerveDrive, driveState, bill]{
        if(*driveState == subsystems::swerve::State::ClimbAlign){
           *driveState = subsystems::swerve::State::HeadingControl;
        } else {
           *driveState = subsystems::swerve::State::ClimbAlign; 
        }
        swerveDrive->SetDefaultCommand(commands::main::Drive(bill, swerveDrive, *driveState));
    },{swerveDrive}).ToPtr();
  }

};



