// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShooterFire extends Command {
  /** Creates a new IntakeConsume. */

  private DoubleSupplier m_SpeedSupplier;
  private final Shooter m_Shooter;
  public ShooterFire(Shooter m_Shooter, DoubleSupplier m_SpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Shooter = m_Shooter;
    this.m_SpeedSupplier = m_SpeedSupplier;
    addRequirements(m_Shooter); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_Shooter.setOutSpeeds(m_SpeedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.setOutSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
