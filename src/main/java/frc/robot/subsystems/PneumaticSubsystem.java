// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticSubsystem extends SubsystemBase {
  
  DoubleSolenoid climbLeft;
  DoubleSolenoid climbRight;
  DoubleSolenoid shooterPiston;
  DoubleSolenoid kickoutPiston;
  CANSparkMax hookMotor;
  //Compressor compress = new Compressor(PneumaticsModuleType.REVPH);
  
  public PneumaticSubsystem() {

    climbLeft = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, Constants.climbLeftPnumaticDeploy, Constants.climbLeftPnumaticRetract);
    climbRight = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, Constants.climbRightPnumaticDeploy, Constants.climbRightPnumaticRetract);
    shooterPiston = new DoubleSolenoid(15 , PneumaticsModuleType.REVPH, Constants.shootPnumaticDeploy, Constants.shootPnumaticRetract);
    kickoutPiston = new DoubleSolenoid(15 , PneumaticsModuleType.REVPH, Constants.kickoutPnumaticDeploy, Constants.kickoutPnumaticRetract);
    hookMotor = new CANSparkMax(Constants.hookMotorPort, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //compress.enableHybrid(0, 120);
    //SmartDashboard.putNumber("Current Pressure", compress.getPressure());
  }

  public void runHookMotor(double speed) {

    hookMotor.set(speed);

  }

  public void climbPistonsUp(){
    climbLeft.set(Value.kForward);
    climbRight.set(Value.kForward);
  }

  public void climbPistonsDown(){
    climbLeft.set(Value.kReverse);
    climbRight.set(Value.kReverse);
  }

  public void kickout(){

    kickoutPiston.set(Value.kForward);

  }

  public void kickoutRetractDEBUG(){

    kickoutPiston.set(Value.kReverse);

  }

  public void climbPistonsToggle(){
    climbLeft.toggle();
    climbRight.toggle();
  }

  public void shooterUp(){
    shooterPiston.set(Value.kForward);
  }

  public void shooterDown(){
    shooterPiston.set(Value.kReverse);
  }
}