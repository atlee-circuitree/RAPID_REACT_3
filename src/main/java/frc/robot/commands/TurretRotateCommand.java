// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretRotateCommand extends CommandBase {
  
  private final TurretSubsystem turret;
  private final LimeLightSubsystem limelight;
  private XboxController xbox;

  public TurretRotateCommand(TurretSubsystem ts, LimeLightSubsystem lim, XboxController xboxController) {

    turret = ts;
    limelight = lim;
    xbox = xboxController;

    addRequirements(turret);

  }
 
  @Override
  public void initialize() {

   limelight.EnableLED();
   

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Check Red/Blue and Green/Blue SparkMax led code
    if(xbox.getLeftTriggerAxis() >= 0.5){
      turret.turnTurret(xbox.getLeftX() / 4);
    }
    else if(limelight.HorizontalOffset() > .2 && limelight.HasValidTarget() == true){
      turret.turnTurret(limelight.HorizontalOffset() / 30);
    }
    else if(limelight.HorizontalOffset() < -.2 && limelight.HasValidTarget() == true){
      turret.turnTurret(limelight.HorizontalOffset() / 30);
    }
    else{
      turret.turnTurret(0);
    }
 
  }

  
  @Override
  public void end(boolean interrupted) {

    turret.turnTurret(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}