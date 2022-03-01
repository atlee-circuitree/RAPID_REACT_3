// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class Pathweaver extends SubsystemBase {
  /** Creates a new Pathweaver. */
  DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  public Pathweaver() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Rename command as needed
  public Command trajectoryA() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                3.0, 3.0)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.SWERVE_KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(.3, 0), new Translation2d(.6, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            1.0, 0, 0, Constants.thetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose, // Functional interface to feed supplier
            Constants.SWERVE_KINEMATICS,

            // Position controllers
            new PIDController(1.0, 0, 0),
            new PIDController(1.0, 0, 0),
            thetaController,
            drivetrain::setSwerveModuleStates,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.killModules());
  }
  
}
