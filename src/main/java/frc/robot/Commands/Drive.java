// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class Drive extends Command {
  /** Creates a new Drive. */
  public final DriveSubsystem m_DriveSubsystem;
  Supplier<Double> m_xVelocitySupplier;
  Supplier<Double> m_yVelcoitySupplier;
  Supplier<Double> m_rotationalVelocitySupplier;
  

  public Drive(DriveSubsystem driveSubsystem, Supplier<Double> xVelocitySupplier, Supplier<Double> yVelocitySupplier, Supplier<Double> rotationalVelocitySupplier) {
    m_xVelocitySupplier = xVelocitySupplier;
    m_yVelcoitySupplier = yVelocitySupplier;
    m_rotationalVelocitySupplier = rotationalVelocitySupplier;
    m_DriveSubsystem = driveSubsystem;
    addRequirements(m_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //scaling factor for velocity
    double velocityScalingFactor = 1;

    //scaling factor for rotational velocity
    double rotationalVelocityScalingFactor = 1;

    //a variable on whether you want to square inputs or not, if you do, only change the boolean to true
    boolean squaredInputs = false;
    velocityScalingFactor = squaredInputs ? velocityScalingFactor : Math.pow(velocityScalingFactor, 2);

    //applies the scaling factors to each of the values from the joysticks which are stored in the suppliers
    double xVelocity = (double)m_xVelocitySupplier.get() * velocityScalingFactor;
    double yVelocity = (double)m_yVelcoitySupplier.get() * velocityScalingFactor;
    double rotationalVelocity = (double)m_rotationalVelocitySupplier.get() * rotationalVelocityScalingFactor;

    //applies the scaled velocities to the drive subsystem
    m_DriveSubsystem.drive(xVelocity, yVelocity, rotationalVelocity, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
