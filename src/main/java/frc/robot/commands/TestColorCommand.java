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

public class TestColorCommand extends CommandBase {
   
  private final FeederSubsystem feeder;
   
  public TestColorCommand(FeederSubsystem fs) {
 
    feeder = fs;
  
    addRequirements(feeder);

  }
 
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (feeder.isBlue() == true) {
        System.out.println("Blue is Seen");
    } else {
        System.out.println("Red: " + feeder.printRed());
        System.out.println("Green: " + feeder.printGreen());
        System.out.println("Blue: " + feeder.printBlue());
    }
 
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