// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends ParallelCommandGroup {
  /** Creates a new AmpScore. */
  public AmpScore(Shooter m_Shooter, Intake m_Intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunCommand(()->m_Shooter.setOutSpeeds(0.4), m_Shooter)
    .alongWith(new RunCommand(()-> m_Intake.setOutSpeeds(0.4), m_Intake)));
  }
}
