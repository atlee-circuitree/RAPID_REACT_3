// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  CANSparkMax feederMotor = null;

  public final ColorSensorV3 colorSensor = new ColorSensorV3(Constants.i2cPort);

  public final static ColorMatch m_colorMatcher = new ColorMatch();

  public static Color Red = new Color(0.561, 0.232, 0.114);

  public static Color Blue = new Color(.3018, .3018, .3686);
 
  Boolean IsRed = false;

  Boolean IsBlue = false;

  Boolean HasBall = false;
   
  public FeederSubsystem() {

    feederMotor = new CANSparkMax(Constants.feederMotorPort, MotorType.kBrushless);

     
  }

  @Override
  public void periodic() {

    if (colorSensor.getProximity() > 800)  {

      HasBall = true;
       
    } else {

      HasBall = false;

    }

    if (colorSensor.getRed() > 550 && colorSensor.getRed() < 5000 && colorSensor.getRed() > colorSensor.getBlue()) {

      IsRed = true;
       
    } else {

      IsRed = false;

  

    }

    if (colorSensor.getBlue() > 550 && colorSensor.getBlue() < 5000 && colorSensor.getBlue() > colorSensor.getRed()) {

      IsBlue = true;
       
    } else {

      IsBlue = false;

    }

    SmartDashboard.putBoolean("Has Ball", HasBall);
    SmartDashboard.putBoolean("Sees Red Ball", IsRed);
    SmartDashboard.putBoolean("Sees Blue Ball", IsBlue);
    SmartDashboard.putNumber("Red Value", colorSensor.getRed());
    SmartDashboard.putNumber("Blue Value", colorSensor.getBlue());
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
  }

  public void runFeeder(double speed) {

    feederMotor.set(speed);

  }

  

  public boolean isRed() {
    
    m_colorMatcher.addColorMatch(Red);

    if (m_colorMatcher.matchColor(Red) != null) {
    return false;
    } else {
    return true;
    }

  }

  public boolean isBlue() {
    
    m_colorMatcher.addColorMatch(Blue);

    if (m_colorMatcher.matchColor(Blue) != null) {
    return false;
    } else {
    return true;
    }

  }

  public int printRed() {

    return colorSensor.getGreen();
  
  }

  public int printGreen() {

  return colorSensor.getGreen();

  }

  public int printBlue() {

    return colorSensor.getBlue();
  
  }

  public void startUSBCamera(){
    CameraServer.startAutomaticCapture();
  }

}

