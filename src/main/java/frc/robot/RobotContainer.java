// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Commands.AmpScore;
import frc.robot.Commands.ClimberClimbWithoutGyro;
import frc.robot.Commands.Drive;
import frc.robot.Commands.IntakeConsume;
import frc.robot.Commands.IntakeEject;
import frc.robot.Commands.ShooterDump;
import frc.robot.Commands.ShooterEject;
import frc.robot.Commands.ArmControl.ClosedLoopArm;
import frc.robot.Commands.ArmControl.OpenLoopArm;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.sql.Driver;
import java.util.List;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Autos m_Autos;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  //Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);
  Arm m_Arm = new Arm();
  Shooter m_Shooter = new Shooter();
  Intake m_Intake = new Intake();
  Climb m_Climb = new Climb();

  


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    
    NamedCommands.registerCommand("RevShooter", new ShooterEject(m_Shooter));
    NamedCommands.registerCommand("IndexToShooter", new IntakeConsume(m_Intake));
    NamedCommands.registerCommand("SetShooterAmp", new ClosedLoopArm(m_Arm, PositionConstants.kSpeakerPosition));

    m_Autos = new Autos(m_robotDrive);
    
    m_Arm.disable();
    
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new Drive(m_robotDrive, 
        ()-> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)));


    

    //TODO: comment out if not using the plain shooter mechanism
    // new Trigger(()-> m_driverController.getRightTriggerAxis() > 0)
    //             .onTrue(new PlainShooterFire(m_PlainShooter, m_driverController::getRightTriggerAxis));





            // () -> m_robotDrive.drive(
            //     -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            //     -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            //     -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            //     true, true),
            // m_robotDrive));
   
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
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new Trigger(()->m_driverController.getStartButton()).onTrue(new InstantCommand(()->m_robotDrive.zeroHeading(), m_robotDrive));


    new Trigger(()-> m_operatorController.getRightY() > 0.2 || m_operatorController.getRightY() < -0.2).whileTrue(new OpenLoopArm(m_Arm, ()-> m_operatorController.getRightY() * 0.8));
    new Trigger(()-> m_operatorController.getLeftTriggerAxis() != 0).whileTrue(new IntakeConsume(m_Intake));
    new Trigger(()-> m_operatorController.getLeftBumper()).whileTrue(new IntakeEject(m_Intake));
    new Trigger(()-> m_operatorController.getRightTriggerAxis() != 0).whileTrue(new ShooterEject(m_Shooter));
    new Trigger(()-> m_operatorController.getRightBumper()).whileTrue(new ShooterDump(m_Shooter));
    new Trigger(()-> m_operatorController.getPOV(0) == 0).whileTrue(
        new AmpScore(m_Shooter, m_Intake))
      .onFalse(
        new RunCommand(()->m_Shooter.setOutSpeeds(0), m_Shooter).alongWith(
        new RunCommand(()-> m_Intake.setOutSpeeds(0), m_Intake)));
    

    new Trigger(()-> m_operatorController.getYButton()).onTrue(new ClosedLoopArm(m_Arm, PositionConstants.kAmpPosition));
    new Trigger(()-> m_operatorController.getAButton()).onTrue(new ClosedLoopArm(m_Arm, PositionConstants.kPickUpPosition));
    new Trigger(()-> m_operatorController.getBButton()).onTrue(new ClosedLoopArm(m_Arm, PositionConstants.kSpeakerPosition));

    new Trigger(()-> DriverStation.isDisabled()).onTrue(new RunCommand(()-> {m_Arm.disable(); m_Arm.setMotorOutputs(0);}, m_Arm));

    

    new Trigger(()-> m_driverController.getRightTriggerAxis() != 0).whileTrue(new ClimberClimbWithoutGyro(m_Climb, ()-> m_driverController.getRightTriggerAxis()));
    //Reverse Climb
    new Trigger(()-> m_driverController.getLeftTriggerAxis() != 0).whileTrue(new ClimberClimbWithoutGyro(m_Climb, ()-> -m_driverController.getLeftTriggerAxis()));


    new Trigger(()-> m_operatorController.getPOV(0) == 180).onTrue(new RunCommand(()-> new IntakeConsume(m_Intake), m_Intake).until(()->m_Intake.isNoteObtained()).andThen(new RunCommand(()-> m_Intake.setOutSpeeds(0), m_Intake)));
    // new JoystickButton(m_driverController, Button.kCircle.value).toggleOnTrue(new ClosedLoopArm(m_Arm, 45));


    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));



    return m_Autos.getAutonomousCommand();
  }
}
