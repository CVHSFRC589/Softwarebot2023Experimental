// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPIDConstants;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhysicalConstants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_PIDController;
  private RelativeEncoder m_encoder;
  private double m_clampedPosition;
  private double m_currentPosition;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_motor = new CANSparkMax(DriveConstants.kArmMotorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_PIDController = m_motor.getPIDController();
    m_currentPosition = m_encoder.getPosition();
    m_clampedPosition = 0;

    m_encoder.setPositionConversionFactor(PhysicalConstants.ARM_GEAR_RATIO);
    // resetEncoders();
    
    m_PIDController.setSmartMotionMaxVelocity(ArmPIDConstants.maxVel, 0);
    m_PIDController.setSmartMotionMinOutputVelocity(ArmPIDConstants.minVel, 0);
    m_PIDController.setSmartMotionMaxAccel(ArmPIDConstants.maxAcc,0);
    m_PIDController.setSmartMotionAllowedClosedLoopError(ArmPIDConstants.allowedErr, 0);
    m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);


    m_PIDController.setP(ArmPIDConstants.kP,0);
    m_PIDController.setI(ArmPIDConstants.kI,0);
    m_PIDController.setD(ArmPIDConstants.kD,0);
    m_PIDController.setIZone(ArmPIDConstants.kIz,0);
    m_PIDController.setFF(ArmPIDConstants.kFF,0);

    //max and min values??
    m_PIDController.setPositionPIDWrappingEnabled(true);
    m_PIDController.setPositionPIDWrappingMaxInput(ArmPIDConstants.PIDWrappingMaxInput);
    m_PIDController.setPositionPIDWrappingMinInput(ArmPIDConstants.PIDWrappingMinInput);
  }
  

  public void resetEncoders(){
    m_encoder.setPosition(0);
  }

  public double getEncoderInches() {
    return m_encoder.getPosition();
  }

  private double clampValue(double x){
    if (x>ArmPhysicalConstants.maxArmValue){
      return ArmPhysicalConstants.maxArmValue;
    }
    else if (x<ArmPhysicalConstants.minArmValue){
      return ArmPhysicalConstants.minArmValue;
    }
    else {
      return x;
    }
  }
  public void incrementPosition(double increment){
    m_currentPosition += increment;
  }
  public double getPosition(){
    m_currentPosition = clampValue(m_currentPosition);
    return m_currentPosition;
  }
  public void setArmPosition(double position){
   // limit position to safe values
  //   m_clampedPosition = clampValue(position);

    m_PIDController.setReference(position, ControlType.kSmartMotion);
  }
  public boolean isInPosition(){

    if(getEncoderInches()>=m_clampedPosition){
      return true;
    }
    else{
      return false;
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position", getEncoderInches());
    SmartDashboard.putNumber("Current Position", m_currentPosition);

    SmartDashboard.putNumber("Clamped Position", m_clampedPosition);
     
    // This method will be called once per scheduler run
  }
}
