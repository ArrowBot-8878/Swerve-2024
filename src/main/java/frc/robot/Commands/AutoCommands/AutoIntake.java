// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */
  private final Intake m_Intake;
  public AutoIntake(Intake m_Intake) {
    this.m_Intake = m_Intake;
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double output = IntakeConstants.kIntakeScalingFactor;
    double output = 1;
    m_Intake.setOutSpeeds(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setOutSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Intake.isNoteObtained();
  }
}
