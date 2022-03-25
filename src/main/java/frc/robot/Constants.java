// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .5715; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */

    public static final double LIMELIGHT_TX_OFFSET = 0;

    public static final double AUTO_MAX_ACCLERATION_METERS_PER_SECOND_SQUARED = 2;
    //public static final double AUTO_MAX_ACCLERATION_METERS_PER_SECOND_SQUARED = 1;
    public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 2;

    
    //public static final double AUTO_MAX_ACCLERATION_METERS_PER_SECOND_SQUARED = 3.4422;
    //public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 7;

    //Values gained from Characterization
    //public static final double kPXController = 0.01809;
    //public static final double kPYController = 0.01809;
    
    //public static final double kPXController = 0.061809;
    //public static final double kPYController = 0.061809;
    public static final double kPXController = 0.067365;
    public static final double kPYController = 0.067365;
    //This was 3, revert back to 3 if something goes wrong
    public static final double kPThetaController = .5;

   //public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(4000, AUTO_MAX_ACCLERATION_METERS_PER_SECOND_SQUARED);
   public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(AUTO_MAX_VELOCITY_METERS_PER_SECOND, AUTO_MAX_ACCLERATION_METERS_PER_SECOND_SQUARED);

    public static final double DRIVETRAIN_WHEELBASE_METERS = .5715; // FIXME Measure and set wheelbase

    public static final double TURRET_ENCODER_CHANGE_TICKS_TO_DEGREES = 1;
 
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(295.037841796875); // FIXME Measure and set front left steer offset
//162.68
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(93.15582275390625); // FIXME Measure and set front right steer offset
//310.60
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(338.280029296875); // FIXME Measure and set back left steer offset
//223.06365966796875
 
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(53.52264404296875); // FIXME Measure and set back right steer offset
//352.606201171875

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    // Old Constants

    public static final I2C.Port i2cPort = I2C.Port.kOnboard;

    //Motors

    public static final int feederMotorPort = 17;
    public static final int topShooterMotorPort = 13;
    public static final int bottomShooterMotorPort = 14;
    public static final int turretMotorPort = 19;
    public static final int hookMotorPort = 18;

    public static final int lowVelocityTop = 5000;
    public static final int lowVelocityBottom = 5000;

    public static final int mediumVelocityTop = 7700;
    public static final int mediumVelocityBottom = 8000;

    public static final int highVelocityTop = 11200;
    public static final int highVelocityBottom = 7800;

    //Pneumatics
    
    public static final int kickoutPnumaticDeploy = 9;
    public static final int kickoutPnumaticRetract = 14;
    public static final int climbLeftPnumaticDeploy = 10;
    public static final int climbLeftPnumaticRetract = 15;
    public static final int climbRightPnumaticDeploy = 0;
    public static final int climbRightPnumaticRetract = 1;
    public static final int shootPnumaticDeploy = 2;
    public static final int shootPnumaticRetract = 3;


    //Auto Variables

    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);

    
}
