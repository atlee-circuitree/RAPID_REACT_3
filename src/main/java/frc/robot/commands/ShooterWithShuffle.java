// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.regex.MatchResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterWithShuffle extends CommandBase {
   
  private final TurretSubsystem turret;
  private final PneumaticSubsystem pneumatic;
  private final LimeLightSubsystem limelight;
  private final FeederSubsystem feeder;
  private double velocity;
  private double bottomVelocity;
  private double distance;
  private long timeout = 1000;
  Timer shooterTime = new Timer();
   

  public ShooterWithShuffle(TurretSubsystem ts, PneumaticSubsystem ps, LimeLightSubsystem ls, FeederSubsystem fs) {
 
    turret = ts;
    pneumatic = ps;
    limelight = ls;
    feeder = fs;
    //SmartDashboard.putNumber("Target Distance", printline)
    addRequirements(turret);
    
  }
 
  @Override
  public void initialize() {
    
    //Limelight stuff
    distance = limelight.getDistanceToTarget();
    //PUT ALGORITHIM HERE
    shooterTime.start();
    shooterTime.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Original time values: 0.5 and 1.2
    if (Math.abs(turret.checkTopMotorWithVelocity() - SmartDashboard.getNumber("Turret Velocity", 9000)) >= 100 && Math.abs(turret.checkBottomMotorWithVelocity() - SmartDashboard.getNumber("Turret Bottom Velocity", 9000)) >= 100) {
 
      turret.runTurretFromSystem(turret.shuffleShooterTop, turret.shuffleShooterBottom);
      System.out.println("Stage 1, not at velocity");
      
    } else if (shooterTime.get() < 2.5) {

      pneumatic.shooterUp();
      System.out.println("Stage 2");
     
    } else {

      pneumatic.shooterDown();
      System.out.println("Stage 3");

    }

    SmartDashboard.putNumber("Timer Time", shooterTime.get());

    //2.9, 1.7 Low Goal 65-110 inches away and 2in from center
    //2.9, 4.0 High Goal 126 inches away
    //2.9, 4.0 High Goal 165 inches away
    //
    //
    //

  }

  @Override
  public void end(boolean interrupted) {
    
    turret.killTurretMotors();
    pneumatic.shooterDown();
     
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if  (shooterTime.get() > 2.5) {

      turret.killTurretMotors();
      return true;

    } else {
      return false;

    } 
 
  }
}
