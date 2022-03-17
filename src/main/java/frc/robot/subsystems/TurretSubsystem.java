// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  public double shuffleShooterTop = -SmartDashboard.getNumber("Turret Velocity", 0);
  public double shuffleShooterBottom = SmartDashboard.getNumber("Turret Bottom Velocity", 0);

  public static String turretDashboard;

  public TurretSubsystem() {
 
    topShootMotor = new TalonSRX(Constants.topShooterMotorPort);
    bottomShootMotor = new TalonSRX(Constants.bottomShooterMotorPort);

    topShootMotor.setNeutralMode(NeutralMode.Coast);
    bottomShootMotor.setNeutralMode(NeutralMode.Coast);

    
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
 
    SmartDashboard.putNumber("Shot Top Velocity Pos/100ms", checkTopMotorWithVelocity());
    SmartDashboard.putNumber("Shot Bottom Velocity Pos/100ms", checkBottomTurretWithVelocity());
 
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
    topShootMotor.set(ControlMode.Velocity, topVelocity);
    bottomShootMotor.set(ControlMode.Velocity, -bottomVelocity);

  }

  public void killTurretMotors(){
    topShootMotor.set(ControlMode.PercentOutput, 0);
    bottomShootMotor.set(ControlMode.PercentOutput, 0);
  }

  public void runTurretWithMPSandShuffle() {

    //topShootMotor.set(ControlMode.Velocity, -metersPerSecondtoVelocity(SmartDashboard.getNumber("Turret Velocity", 0)));
    //bottomShootMotor.set(ControlMode.Velocity, metersPerSecondtoVelocity(SmartDashboard.getNumber("Turret Bottom Velocity", 0)));

  }

  public void runTurretFromSystem(double topVar, double bottomVar) {

    topShootMotor.set(ControlMode.Velocity, -SmartDashboard.getNumber("Turret Velocity", 0));
    bottomShootMotor.set(ControlMode.Velocity, SmartDashboard.getNumber("Turret Bottom Velocity", 0));

  }

  public double checkTopMotorWithVelocity() {
    //Cose SZ shoot = Bottom * 1.3
    return -topShootMotor.getSelectedSensorVelocity();
    
  }

  public double checkBottomMotorWithVelocity() {
    //Close SZ shoot = Bottom * 1.3
    return bottomShootMotor.getSelectedSensorVelocity();
    
  }

  public double checkBottomTurretWithVelocity() {
    //Close SZ shoot = Bottom * 1.3
    return bottomShootMotor.getSelectedSensorVelocity();
    
  }

  public void turnTurret(double speed) {

    turretMotor.set(speed);
  
  }
  
  public double metersPerSecondtoVelocity(double metersPerSecond){
    
    double degreesPerSecond = (metersPerSecond/0.1016)*(180/Math.PI);
    double positionChangePer100ms = (degreesPerSecond * 44.9)/10;

    return positionChangePer100ms;
  }

  public double[] distanceToShooterMatrix(double distanceIn){

    //ENTER VALUES IN LOW TO HIGH DISTANCE
    double[] distanceList = {2, 2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double[] topVelocityList = {2.3, 2.3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double[] bottomVelocityList = {3.8, 3.8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    for(int i = 0; i < distanceList.length; i++){
      
      try{
        //If the distance in is bigger than the current value being checked, but smaller than the next,
        //it means that the distance in is between one of those values, meaning that we will return one of those 2 points
        if(distanceIn > distanceList[i] && distanceIn < distanceList[i+1]){

          //If the distance between the distance in and the 1st value in consideration is smaller than the distance between
          //the distance in and the 2nd value in consideration, return the velocities corresponding to the 1st value
          if(Math.abs(distanceIn - distanceList[i]) < Math.abs(distanceIn - distanceList[i+1])){
            double[] returnValues = {topVelocityList[i], bottomVelocityList[i]};
            return returnValues;
          }
          //Otherwise return the velocities corresponding to the 2nd value
          else{
            double[] returnValues = {topVelocityList[i+1], bottomVelocityList[i+1]};
            return returnValues;
          }

        }
      }
      //If we somehow get a distance larger than any of the ones we have in the table (causing an error),
      //output the largest value we have
      catch(ArrayIndexOutOfBoundsException exception){
        double[] returnValues = {topVelocityList[topVelocityList.length], bottomVelocityList[bottomVelocityList.length]};
        return returnValues;
      }

    }
    
    return null;

  }
  
 
}  