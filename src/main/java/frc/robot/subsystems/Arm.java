// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends PIDSubsystem {
  /** Creates a new Arm. */
  private final CANSparkMax m_LeftArmMotor;
  private final CANSparkMax m_RightArmMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  private double m_PreviousDegreeVelocity = 0;
  private double m_DegreeAcceleration = 0;
  private final double m_TimePeriod = 0.02;

  private double m_degreesPerSecond;
  


  public Arm() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));

    m_LeftArmMotor = new CANSparkMax(ArmConstants.kLeftArmMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_RightArmMotor = new CANSparkMax(ArmConstants.kRightArmMotor, CANSparkLowLevel.MotorType.kBrushless);

    m_LeftArmMotor.setIdleMode(IdleMode.kBrake);
    m_RightArmMotor.setIdleMode(IdleMode.kBrake);

    m_LeftArmMotor.setSmartCurrentLimit(ArmConstants.kMaxAmps);
    m_RightArmMotor.setSmartCurrentLimit(ArmConstants.kMaxAmps);

    m_AbsoluteEncoder = m_RightArmMotor.getAbsoluteEncoder(Type.kDutyCycle);  
    m_AbsoluteEncoder.setPositionConversionFactor(360);

    m_LeftArmMotor.burnFlash();
    m_RightArmMotor.burnFlash();

    super.m_controller.setTolerance(ArmConstants.angleTolerance);
    m_controller.enableContinuousInput(0, 360);
    this.disable();
  }

  @Override
  public void useOutput(double PIDOutput, double setpoint) {
    // Use the output here
    if((m_AbsoluteEncoder.getPosition() < 180 || m_AbsoluteEncoder.getPosition() > 355) && m_enabled) {
      double feedForward = ArmConstants.kHoldPosition * Math.cos(getMeasurement()); 
      double motorOutput = feedForward + PIDOutput;
      if (motorOutput > 0.4){
        motorOutput = 0.4;
      } else if (motorOutput < -0.4) {
        motorOutput = -0.4;
      }
      if(ArmConstants.kIsPidInverted){
        m_LeftArmMotor.set(-motorOutput);
        m_RightArmMotor.set(motorOutput);
      } else{
        m_LeftArmMotor.set(motorOutput);
        m_RightArmMotor.set(-motorOutput);
      }
    } else {
      m_LeftArmMotor.set(0);
      m_RightArmMotor.set(0);
    }
    System.out.println("PID is On");

  }

  @Override
  public void periodic(){
    m_DegreeAcceleration = (m_AbsoluteEncoder.getVelocity() - m_PreviousDegreeVelocity) / m_TimePeriod;
    m_PreviousDegreeVelocity = m_AbsoluteEncoder.getVelocity();
    m_AbsoluteEncoder.getVelocity();
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement()), getSetpoint());
    }
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_AbsoluteEncoder.getPosition();
  }


  public void setMotorOutputs(double output){
    if (output > 0.4){
      output = 0.4;
    } else if (output < -0.4) {
      output = -0.4;
    }
      if(ArmConstants.kIsMovingForwardActuallyMovingBack){
        m_LeftArmMotor.set(-output);
        m_RightArmMotor.set(output);
      } else{
        m_LeftArmMotor.set(output);
        m_RightArmMotor.set(-output);
      }
  
  }
}
