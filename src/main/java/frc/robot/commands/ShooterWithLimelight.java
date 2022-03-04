// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterWithLimelight extends CommandBase {
   
  private final TurretSubsystem turret;
  private final PneumaticSubsystem pneumatic;
  private final LimeLightSubsystem limelight;
  private final FeederSubsystem feeder;
  private double velocity;
  private double distance;
  private long timeout = 1000;

  public ShooterWithLimelight(double targetVelocity ,TurretSubsystem ts, PneumaticSubsystem ps, LimeLightSubsystem ls, FeederSubsystem fs) {
 
    turret = ts;
    pneumatic = ps;
    limelight = ls;
    feeder = fs;
    velocity = targetVelocity;
    addRequirements(turret);

  }
 
  @Override
  public void initialize() {
    
    //Limelight stuff
    distance = limelight.getDistanceToTarget();
    //PUT ALGORITHIM HERE
 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    turret.runTurretWithVelocity(velocity);
    Timer.delay(.5);
    pneumatic.shooterUp();
    Timer.delay(1);
    turret.runTurretWithVelocity(0);
    pneumatic.shooterDown();
    Timer.delay(1);
 
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
