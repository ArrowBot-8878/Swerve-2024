// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax m_intakeMotor;
  private final DigitalInput m_lightSensor;
  public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    m_lightSensor = new DigitalInput(9);
  }

  public void setOutSpeeds(double relativeSpeed){
    if (IntakeConstants.invertIntake) {
      relativeSpeed *= -1;
    }
    m_intakeMotor.set(relativeSpeed);
  }

  public boolean isNoteObtained(){
    return m_lightSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(isNoteObtained());
  }
}
