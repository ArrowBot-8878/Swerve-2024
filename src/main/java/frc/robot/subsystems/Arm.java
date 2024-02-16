// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends PIDSubsystem {
  /** Creates a new Arm. */
  private final CANSparkMax m_LeftArmMotor;
  private final CANSparkMax m_RightArmMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  


  public Arm() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));

    m_LeftArmMotor = new CANSparkMax(ArmConstants.kLeftArmMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_RightArmMotor = new CANSparkMax(ArmConstants.kRightArmMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_LeftArmMotor.setSmartCurrentLimit(ArmConstants.kMaxAmps);
    m_AbsoluteEncoder = m_RightArmMotor.getAbsoluteEncoder(Type.kDutyCycle);  
    m_AbsoluteEncoder.setPositionConversionFactor(360);
    
    super.m_controller.setTolerance(ArmConstants.angleTolerance);
    
  }

  @Override
  public void useOutput(double PIDOutput, double setpoint) {
    // Use the output here
    if(m_AbsoluteEncoder.getPosition() < 90) {
      double feedForward = ArmConstants.kHoldPosition * Math.cos(getMeasurement()); 
      double motorOutput = feedForward + PIDOutput;
      if(ArmConstants.kIsPidInverted){
        m_LeftArmMotor.set(-motorOutput);
        m_RightArmMotor.set(motorOutput);
      } else{
        m_LeftArmMotor.set(motorOutput);
        m_RightArmMotor.set(-motorOutput);
      }
    }

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_AbsoluteEncoder.getPosition();
  }

  public void setMotorOutputs(double output){
      if(ArmConstants.kIsMovingForwardActuallyMovingBack){
        m_LeftArmMotor.set(-output);
        m_RightArmMotor.set(output);
      } else{
        m_LeftArmMotor.set(output);
        m_RightArmMotor.set(-output);
      }
  }

}
