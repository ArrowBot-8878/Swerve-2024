// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax m_LeftMotor;
  private final CANSparkMax m_RightMotor;
  public Shooter() {
    m_LeftMotor = new CANSparkMax(ShooterConstants.kLeftMotorCAN, CANSparkLowLevel.MotorType.kBrushless);
    m_RightMotor = new CANSparkMax(ShooterConstants.kRightMotorCAN, CANSparkLowLevel.MotorType.kBrushless);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOutSpeeds(double relativeSpeed){
    double m_LeftMotorOutput = relativeSpeed;
    double m_RightMotorOutput = relativeSpeed;

    if (ShooterConstants.isInverted){
      m_LeftMotorOutput *= -1;
      m_RightMotorOutput *= -1;
    }

    m_LeftMotor.set(m_LeftMotorOutput);
    m_RightMotor.set(m_RightMotorOutput);
  }
}
