// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.PlainShooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PlainShooter;

public class PlainShooterFire extends Command {
  private final PlainShooter m_PlainShooter;
  private final Supplier<Double> m_SpeedSupplier;
  /** Creates a new PlainShooterFire. */

  public PlainShooterFire(PlainShooter m_plainShooter, Supplier<Double> m_SpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PlainShooter = m_plainShooter;
    this.m_SpeedSupplier = m_SpeedSupplier;
    addRequirements(m_plainShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double output = m_SpeedSupplier.get();
    m_PlainShooter.setMotorOutput(output);
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
