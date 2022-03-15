// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.dependents.AbsoluteEncoder;

public class TurretSubsystem extends SubsystemBase {
   
  TalonSRX topShootMotor = null;
  TalonSRX bottomShootMotor = null;
  CANSparkMax turretMotor = null;
  RelativeEncoder turretEncoder = null;

  public static String turretDashboard;

  public TurretSubsystem() {
 
    topShootMotor = new TalonSRX(Constants.topShooterMotorPort);
    bottomShootMotor = new TalonSRX(Constants.bottomShooterMotorPort);
    
    turretMotor = new CANSparkMax(Constants.turretMotorPort, MotorType.kBrushless);

    turretEncoder = turretMotor.getEncoder(Type.kHallSensor, 42);
  
  }

  @Override
  public void periodic() {

    
    SmartDashboard.putNumber("Turret Angle", getTurretEncoder());
    
    double smartVelocity = SmartDashboard.getNumber("Turret Velocity", 0);
    SmartDashboard.putNumber("Turret Velocity", smartVelocity);
    double smartBottomMotorMod = SmartDashboard.getNumber("Turret Bottom Velocity", 1);
    SmartDashboard.putNumber("Turret Bottom Velocity", smartBottomMotorMod);
    SmartDashboard.putNumber("Shot Top Velocity", metersPerSecondConversion(checkTurretWithVelocity()));
    SmartDashboard.putNumber("Shot Bottom Velocity", metersPerSecondConversion(checkBottomTurretWithVelocity()));
 
  }

  protected void useOutput(double output, double setpoint) {
    turretMotor.set(output);
  }

  
  public double getTurretEncoder() {

    return turretEncoder.getPosition() / Constants.TURRET_ENCODER_CHANGE_TICKS_TO_DEGREES;

  }

  public double getMeasurement() {
    return 0;
  }

  public void runTurretWithVelocity(double topVelocity, double bottomVelocity) {
    //Close SZ shoot = Bottom * 1.3
    topShootMotor.set(ControlMode.Velocity, -topVelocity);
    bottomShootMotor.set(ControlMode.Velocity, bottomVelocity);

  }

  public void runTurretWithMPHandSmart() {

    topShootMotor.set(ControlMode.Velocity, -metersPerSecondConversion(SmartDashboard.getNumber("Turret Velocity", 0)));
    bottomShootMotor.set(ControlMode.Velocity, metersPerSecondConversion(SmartDashboard.getNumber("Turret Bottom Velocity", 0)));

  }

  public double checkTurretWithVelocity() {
    //Close SZ shoot = Bottom * 1.3
    return topShootMotor.getSelectedSensorVelocity();
    
  }

  public double checkBottomTurretWithVelocity() {
    //Close SZ shoot = Bottom * 1.3
    return bottomShootMotor.getSelectedSensorVelocity();
    
  }


  public double getVelocity() {

    double velocity;
    velocity = topShootMotor.getSelectedSensorVelocity();

    return velocity;

  }

  public void turnTurret(double speed) {

    turretMotor.set(speed);
  
  }
  
  public double metersPerSecondConversion(double metersPerSecond){
    
    double degreesPerSecond = (metersPerSecond/0.1016)*(180/Math.PI);
    double positionChangePer100ms = (degreesPerSecond * 44.9)/10;

    return positionChangePer100ms;
  }
 
}  