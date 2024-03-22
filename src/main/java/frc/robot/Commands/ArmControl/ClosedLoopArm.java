// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ArmControl;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ClosedLoopArm extends InstantCommand {
  /** Creates a new ClosedLoopArm. */
  private final Arm m_Arm;
  private final double m_targetSetpoint;
  public ClosedLoopArm(Arm arm, double targetSetpoint) {
    m_Arm = arm;
    m_targetSetpoint = targetSetpoint;
    addRequirements(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.enable();
    m_Arm.setSetpoint(m_targetSetpoint);
    System.out.println("start closed looping");
  }
}
