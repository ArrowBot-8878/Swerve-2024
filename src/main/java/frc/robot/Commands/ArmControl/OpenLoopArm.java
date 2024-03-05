// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ArmControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class OpenLoopArm extends Command {
  /** Creates a new OpenLoopArm. */
  private final Supplier<Double> m_OutputSupplier;
  private final Arm m_Arm;
  public OpenLoopArm(Arm arm, Supplier<Double> outputSupplier) {
    m_OutputSupplier = outputSupplier;
    m_Arm = arm;
    addRequirements(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double output = m_OutputSupplier.get();
    m_Arm.setMotorOutputs(output);
    System.out.println("Open Looping");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_Arm.setMotorOutputs(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
