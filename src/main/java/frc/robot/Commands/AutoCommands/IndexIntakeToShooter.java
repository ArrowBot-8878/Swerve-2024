// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IndexIntakeToShooter extends Command {
  /** Creates a new IntakeConsume. */
  private final Intake m_intake;

  private double m_output;

  public IndexIntakeToShooter(Intake m_intake, double m_output) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_output = m_output;
    this.m_intake = m_intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double output = IntakeConstants.kIntakeScalingFactor;
    m_intake.setOutSpeeds(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setOutSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
