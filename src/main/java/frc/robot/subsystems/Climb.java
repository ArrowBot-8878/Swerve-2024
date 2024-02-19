// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private CANSparkMax m_LeftClimbMotor;
  private CANSparkMax m_RightClimbMotor;

  public Climb() {
  m_LeftClimbMotor = new CANSparkMax(ClimberConstants.LeftArmMotor, CANSparkLowLevel.MotorType.kBrushless);
  m_RightClimbMotor = new CANSparkMax(ClimberConstants.kRightArmMotor, CANSparkLowLevel.MotorType.kBrushless);
  }

  public void setMotorOutput(double leftOutput, double rightOutput){
    m_LeftClimbMotor.set(leftOutput);
    m_RightClimbMotor.set(rightOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
