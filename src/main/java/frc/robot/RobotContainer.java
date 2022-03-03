// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import java.util.List;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbPistonCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.KickoutPistonCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunFeederAuto;
import frc.robot.commands.RunHookCommand;
import frc.robot.commands.ShooterWithLimelight;
import frc.robot.commands.TestColorCommand;
import frc.robot.commands.TurretRotateCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Pathweaver;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final PneumaticSubsystem m_pneumaticSubsystem = new PneumaticSubsystem();
  private final LimeLightSubsystem m_limelightSubsystem = new LimeLightSubsystem();
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
   
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller2 = new XboxController(1);
  private final Joystick m_controller3 = new Joystick(2);
 
  //Single Commands
  private final TurretRotateCommand m_turretRotateCommand = new TurretRotateCommand(m_turretSubsystem, m_limelightSubsystem, m_controller2);
  private final DefaultDriveCommand m_driveCommand = new DefaultDriveCommand(m_drivetrainSubsystem,() -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,() -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,() -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
  private final ShooterWithLimelight m_shootCommand = new ShooterWithLimelight(4000, m_turretSubsystem, m_pneumaticSubsystem, m_limelightSubsystem, m_feederSubsystem);
  private final TestColorCommand m_colorTest = new TestColorCommand(m_feederSubsystem);
  private final KickoutPistonCommand m_kickoutCommand = new KickoutPistonCommand(m_pneumaticSubsystem);
  private final RunFeederAuto m_runFeederAuto = new RunFeederAuto(.5, m_feederSubsystem, m_pneumaticSubsystem);
  public Command m_feederCommand(double speed) {
    Command m_feedCommand = new RunFeeder(speed, m_feederSubsystem, m_pneumaticSubsystem);
    return m_feedCommand;
  }
  public Command m_hookCommand(double speed) {
    Command m_hookCommand = new RunHookCommand(speed, m_pneumaticSubsystem);
    return m_hookCommand;
  }
  public Command m_climbPistonCommand(boolean isUp) {
    Command m_pneumaticCommand = new ClimbPistonCommand(isUp, m_pneumaticSubsystem);
    return m_pneumaticCommand;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    PerpetualCommand DriveWithTurret = new PerpetualCommand(m_driveCommand.alongWith(m_turretRotateCommand)); 

    m_drivetrainSubsystem.zeroGyroscope();
    
    m_drivetrainSubsystem.setDefaultCommand(DriveWithTurret);
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    //P1 BUTTONS
    JoystickButton DriverA = new JoystickButton(m_controller, XboxController.Button.kA.value);
    JoystickButton DriverB = new JoystickButton(m_controller, XboxController.Button.kB.value);
    
    DriverA.whileHeld(m_feederCommand(-.5));
    DriverB.whileHeld(m_feederCommand(.5));

    //P2 BUTTONS
    JoystickButton OperatorA = new JoystickButton(m_controller2, XboxController.Button.kA.value);
    JoystickButton OperatorB = new JoystickButton(m_controller2, XboxController.Button.kB.value);
    
    //OperatorA.whenPressed(m_shootCommand);
    //DriverB.whileHeld(m_colorTest);

    //Fightstick Buttons
    JoystickButton FightShare = new JoystickButton(m_controller3, 7);
    JoystickButton FightOption = new JoystickButton(m_controller3, 8);
    JoystickButton FightL3 = new JoystickButton(m_controller3, 9);
    JoystickButton FightR3 = new JoystickButton(m_controller3, 10);
    JoystickButton FightL1 = new JoystickButton(m_controller3, 5);

    FightL1.whenPressed(m_kickoutCommand);
    FightShare.whenPressed(m_climbPistonCommand(true));
    FightOption.whenPressed(m_climbPistonCommand(false));
    FightL3.whileHeld(m_hookCommand(1));
    FightR3.whileHeld(m_hookCommand(-1));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(m_drivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ACCLERATION_METERS_PER_SECOND_SQUARED).setKinematics(Constants.SWERVE_KINEMATICS);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),

    List.of(
    new Translation2d(-3, 0)
     
    ), new Pose2d(-6, 0, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

    PIDController xController = new PIDController(Constants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kPThetaController, 0, 0, Constants.thetaControllerConstraints);
    thetaController.enableContinuousInput(Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    trajectory,
    m_drivetrainSubsystem::getPose,
    Constants.SWERVE_KINEMATICS,
    xController,
    yController,
    thetaController,
    m_drivetrainSubsystem::setSwerveModuleStates, 
    m_drivetrainSubsystem);

    SequentialCommandGroup DriveAuto = new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> m_drivetrainSubsystem.killModules()));

    SequentialCommandGroup Auto = new SequentialCommandGroup(m_kickoutCommand.withTimeout(1), m_runFeederAuto.withTimeout(1),
    DriveAuto, m_feederCommand(0).withTimeout(1), m_shootCommand);

    return Auto;

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
