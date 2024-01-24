// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PlainShooterConstants;

public class PlainShooter extends SubsystemBase {
  /** Creates a new PlainShooter. */
  private final CANSparkMax m_ShooterMotor;
  public PlainShooter() {
    m_ShooterMotor= new CANSparkMax(PlainShooterConstants.kShooterMotor, CANSparkLowLevel.MotorType.kBrushless);
  }

  public void setMotorOutput(double desiredOutput){
    desiredOutput *= PlainShooterConstants.kScalingFactor;
    m_ShooterMotor.set(desiredOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
