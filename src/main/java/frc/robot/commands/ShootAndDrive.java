// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootAndDrive extends CommandBase {
   
  private final TurretSubsystem turret;
  private final PneumaticSubsystem pneumatic;
  private final LimeLightSubsystem limelight;
  private final FeederSubsystem feeder;
  private double topVelocity = 0;
  private double bottomVelocity = 0;
  private XboxController xboxController;
  private double distance;
  private long timeout = 1000;
  Timer shooterTime = new Timer();
   

  public ShootAndDrive(TurretSubsystem ts, PneumaticSubsystem ps, LimeLightSubsystem ls, FeederSubsystem fs, XboxController xbox) {
 
    turret = ts;
    pneumatic = ps;
    limelight = ls;
    feeder = fs;
    addRequirements(turret);
    
  }
 
  @Override
  public void initialize() {
    
    
    shooterTime.start();
    shooterTime.reset();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    //Limelight stuff
    distance = limelight.getDistanceToTarget();
    //PUT ALGORITHIM HERE

    //Run turret
    if (Math.abs(turret.checkTopMotorWithVelocity() - turret.metersPerSecondtoVelocity(topVelocity)) >= 100 && shooterTime.get() < 0.5) {
 
      turret.runTurretWithVelocity(topVelocity, bottomVelocity);
      
    } else if (shooterTime.get() < 1.2) {

      pneumatic.shooterUp();
     
    } else {

      pneumatic.shooterDown();

    }

    //Kill Motors
    if  (shooterTime.get() > 1.25) {

      turret.killTurretMotors();

    }
 
  

    SmartDashboard.putNumber("Timer Time", shooterTime.get());

  }

  @Override
  public void end(boolean interrupted) {
     
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
