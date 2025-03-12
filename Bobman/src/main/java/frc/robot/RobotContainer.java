// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
//import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.ElevatorDownCmd;
import frc.robot.commands.ElevatorUpCmd;
import frc.robot.commands.RaiseFunnelCmd;
import frc.robot.commands.ResetClimberCmd;
import frc.robot.commands.ResetFunnelCmd;
import frc.robot.commands.ScoreCmd;
import frc.robot.commands.SetEndEffectorCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  //private final SendableChooser<Command> autoChooser;

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final FunnelSubsystem funnelSubsystem = new FunnelSubsystem();
  private final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();

  private final Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureDriverBindings();

    configureOperatorBinding();

    //autoChooser = AutoBuilder.buildAutoChooser();

    //SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, DriveConstants.modeValue),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureDriverBindings() {
/*X Button */    new JoystickButton(m_driverController, Button.kSquare.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    // slowest mode
/*L1 */    new JoystickButton(m_driverController, Button.kL1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.changeModeValue(1),
            m_robotDrive));
    // slow mode
/*R1 */    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.changeModeValue(2),
            m_robotDrive));
    // Climber Controls
/*Start */    new JoystickButton(m_driverController, Button.kOptions.value).whileTrue(new ClimbCmd(climberSubsystem, Constants.DriveConstants.climberMotorSpeed));
/*Back */    new JoystickButton(m_driverController, Button.kShare.value).whileTrue(new ResetClimberCmd(climberSubsystem, -Constants.DriveConstants.resetClimberMotorSpeed));
    // Funnel Controls
/*Y Button */    new JoystickButton(m_driverController, Button.kTriangle.value).whileTrue(new RaiseFunnelCmd(funnelSubsystem, Constants.DriveConstants.funnelMotorSpeed));
/*A Button */    new JoystickButton(m_driverController, Button.kCross.value).whileTrue(new ResetFunnelCmd(funnelSubsystem, Constants.DriveConstants.resetFunnelMotorSpeed));
    // Score Controls
/*L2 */    new JoystickButton(m_driverController, Button.kL2.value).whileTrue(new ScoreCmd(endEffectorSubsystem, Constants.DriveConstants.scoreMotorFullSpeed));
/*R2 */    new JoystickButton(m_driverController, Button.kR2.value).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, Constants.DriveConstants.scoreMotorSlowSpeed));
  }

  private void configureOperatorBinding() {
    // reef level 3
    new JoystickButton(m_operatorController, 1).whileTrue(new ElevatorUpCmd(elevatorSubsystem, Constants.DriveConstants.elevatorMotorSpeed));
    // reef level 2 
    new JoystickButton(m_operatorController, 3).whileTrue(new ElevatorUpCmd(elevatorSubsystem, Constants.DriveConstants.elevatorMotorSpeed));
    // reef level 1
    new JoystickButton(m_operatorController, 6).whileTrue(new ElevatorUpCmd(elevatorSubsystem, Constants.DriveConstants.elevatorMotorSpeed));

    // reef level 3
/*X Button */    if (elevatorSubsystem.getElevatorEncoderValue() >= 2.5 && m_operatorController.getRawButton(1)){
      new ElevatorUpCmd(elevatorSubsystem, Constants.DriveConstants.elevatorMotorSpeed);
    }
    // reef level 2
/*B Button */    else if (elevatorSubsystem.getElevatorEncoderValue() >= 1.8 && m_operatorController.getRawButton(3)){
      new ElevatorUpCmd(elevatorSubsystem, Constants.DriveConstants.elevatorMotorSpeed);
    }
    // reef level 1
/*R1 */    else if (elevatorSubsystem.getElevatorEncoderValue() >= 1.3 && m_operatorController.getRawButton(6)){
      new ElevatorUpCmd(elevatorSubsystem, Constants.DriveConstants.elevatorMotorSpeed);
    } else{
      new ElevatorUpCmd(elevatorSubsystem, 0);
    }
    // Elevator Controls
/*Y Button*/  new JoystickButton(m_operatorController, 4).whileTrue(new ElevatorUpCmd(elevatorSubsystem, -Constants.DriveConstants.elevatorMotorSpeed));
/*A Button */    new JoystickButton(m_operatorController, 2).whileTrue(new ElevatorDownCmd(elevatorSubsystem, Constants.DriveConstants.elevatorMotorSpeed));
    // Climber Controls
/*Start Button*/    new JoystickButton(m_operatorController, 10).whileTrue(new ClimbCmd(climberSubsystem, Constants.DriveConstants.climberMotorSpeed));
/*Back Button */    new JoystickButton(m_operatorController, 9).whileTrue(new ResetClimberCmd(climberSubsystem, -Constants.DriveConstants.resetClimberMotorSpeed));
    // Funnel Controls
/*L2 */    new JoystickButton(m_operatorController, 7).whileTrue(new RaiseFunnelCmd(funnelSubsystem, Constants.DriveConstants.funnelMotorSpeed));
/*R2 */    new JoystickButton(m_operatorController, 8).whileTrue(new ResetFunnelCmd(funnelSubsystem, Constants.DriveConstants.resetFunnelMotorSpeed));
    // Score Controls
/*L3 */    new JoystickButton(m_operatorController, 11).whileTrue(new ScoreCmd(endEffectorSubsystem, Constants.DriveConstants.scoreMotorFullSpeed));
/*R3 */    new JoystickButton(m_operatorController, 12).whileTrue(new SetEndEffectorCmd(endEffectorSubsystem, Constants.DriveConstants.scoreMotorSlowSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetPose(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

  }*/

  //public Command getAutonomousPathCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path\
    
    //return new PathPlannerAuto("New Auto");
    //return autoChooser.getSelected();
  //}
}