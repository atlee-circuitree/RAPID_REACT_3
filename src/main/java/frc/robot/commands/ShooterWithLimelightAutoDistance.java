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
import frc.robot.LaunchVelocity;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterWithLimelightAutoDistance extends CommandBase {
   
  private final TurretSubsystem turret;
  private final PneumaticSubsystem pneumatic;
  private final LimeLightSubsystem limelight;
  private final FeederSubsystem feeder;
  public double velocity;
  public double bottomVelocity;
  private double distance;
  private long timeout = 1000;
  Timer shooterTime = new Timer();

  double startingTime = shooterTime.get();   

  public ShooterWithLimelightAutoDistance(TurretSubsystem ts, PneumaticSubsystem ps, LimeLightSubsystem ls, FeederSubsystem fs) {
 
    turret = ts;
    pneumatic = ps;
    limelight = ls;
    feeder = fs;
    addRequirements(turret);
    
  }
 
  @Override
  public void initialize() {
    
    //Limelight stuff
    distance = limelight.getDistanceToTarget();
 
    int roundedDistance = (int) Math.round(distance * 10);
    
    LaunchVelocity[] launchVelocityArray = turret.getDistanceToVelocityArray();

    velocity = launchVelocityArray[roundedDistance].topMotorVelocity;

    bottomVelocity = launchVelocityArray[roundedDistance].bottomMotorVelocity;

    SmartDashboard.putNumber("Real rounded distance", roundedDistance);

    //PUT ALGORITHIM HERE
    shooterTime.start();
    shooterTime.reset();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(velocity) >= Math.abs(turret.returnTopMotorWithVelocity()) - 50 && Math.abs(bottomVelocity) >= Math.abs(turret.returnBottomMotorWithVelocity()) - 50) {
    
      turret.runTurretWithVelocity(velocity, bottomVelocity);
      SmartDashboard.putNumber("Target Top Speed", velocity);
      SmartDashboard.putNumber("Target Bottom Speed", bottomVelocity);
      System.out.println("Stage 1, not at velocity");
      
    } else if (shooterTime.get() < 1.5) {

      pneumatic.shooterUp();
      System.out.println("Stage 2");
     
    } else {

      pneumatic.shooterDown();
      System.out.println("Stage 3");

    }
  }

  @Override
  public void end(boolean interrupted) {

    turret.killTurretMotors();
    pneumatic.shooterDown();
     
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if  (shooterTime.get() > 2) {

      turret.runTurretWithVelocity(0, 0);
      return true;

    } else {
      return false;

    } 
 
  }
}
