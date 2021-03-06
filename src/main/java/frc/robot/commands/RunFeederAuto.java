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

public class RunFeederAuto extends CommandBase {
   
  private final FeederSubsystem feeder;
  private final PneumaticSubsystem pnuematic;
  private double targetSpeed; 

  public RunFeederAuto(double speed, FeederSubsystem fs, PneumaticSubsystem ps) {
 
    feeder = fs;
    pnuematic = ps;
    targetSpeed = speed;
    addRequirements(feeder);

  }
 
  @Override
  public void initialize() {
    
    pnuematic.kickout();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    feeder.runFeeder(targetSpeed);
 
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