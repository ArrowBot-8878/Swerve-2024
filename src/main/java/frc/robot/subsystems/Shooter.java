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
  private final CANSparkMax m_topMotor;
  private final CANSparkMax m_bottomMotor;
  public Shooter() {
    m_topMotor = new CANSparkMax(ShooterConstants.kTopMotorCAN, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomMotor = new CANSparkMax(ShooterConstants.kBottomMotorCAN, CANSparkLowLevel.MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOutSpeeds(double relativeSpeed){
    double topOutput = relativeSpeed;
    double bottomOutput = relativeSpeed * -1;

    if (ShooterConstants.isInverted){
      topOutput *= -1;
      bottomOutput *= -1;
    }

    m_topMotor.set(topOutput);
    m_bottomMotor.set(bottomOutput);
  }
}
