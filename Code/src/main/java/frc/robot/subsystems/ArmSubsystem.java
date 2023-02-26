// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPIDConstants;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhysicalConstants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_PIDController;
  private SparkMaxLimitSwitch m_upperlimitswitch;
  private SparkMaxLimitSwitch m_lowerlimitswitch;
  private RelativeEncoder m_encoder;
  private double m_clampedPosition;
  private double m_currentPosition;
  private boolean m_fixedposition;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_motor = new CANSparkMax(IDConstants.kArmMotorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_motor.setSmartCurrentLimit(ArmPhysicalConstants.maxArmAmp);
    m_PIDController = m_motor.getPIDController();
    m_currentPosition = m_encoder.getPosition();
    m_fixedposition = false;
    m_clampedPosition = 0;
    m_motor.setInverted(true);
    m_motor.setIdleMode(IdleMode.kBrake);

    m_upperlimitswitch = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_lowerlimitswitch = m_motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_encoder.setPositionConversionFactor(PhysicalConstants.ARM_GEAR_RATIO);
    // resetEncoders();

    m_PIDController.setSmartMotionMaxVelocity(ArmPIDConstants.maxVel, 0);
    m_PIDController.setSmartMotionMinOutputVelocity(ArmPIDConstants.minVel, 0);
    m_PIDController.setSmartMotionMaxAccel(ArmPIDConstants.maxAcc, 0);
    m_PIDController.setSmartMotionAllowedClosedLoopError(ArmPIDConstants.allowedErr, 0);
    m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_PIDController.setP(ArmPIDConstants.kP, 0);
    m_PIDController.setI(ArmPIDConstants.kI, 0);
    m_PIDController.setD(ArmPIDConstants.kD, 0);
    m_PIDController.setIZone(ArmPIDConstants.kIz, 0);
    m_PIDController.setFF(ArmPIDConstants.kFF, 0);

    // max and min values??

    // THESE do NOT change anything....
    // m_PIDController.setPositionPIDWrappingEnabled(true);
    // m_PIDController.setPositionPIDWrappingMaxInput(ArmPIDConstants.PIDWrappingMaxInput);
    // m_PIDController.setPositionPIDWrappingMinInput(ArmPIDConstants.PIDWrappingMinInput);
    // m_upperlimitswitch.enableLimitSwitch(true);

  }
  // ==========================================SENSOR METHODS========================================== \\

  public double getEncoderDeg() {
    return m_encoder.getPosition()*360;
  }
  
  public boolean isLowerLimitSwitchPressed(){
    return m_lowerlimitswitch.isPressed();
  }
  public boolean isUpperLimitSwitchPressed(){
    return m_upperlimitswitch.isPressed();
  }

  public void resetEncoders() {
    m_encoder.setPosition(0);
  }
 

  public double getEncoderInches() {
    return m_encoder.getPosition();
  }

  

  
  public double clampValue(double x) {
    if (x > ArmPhysicalConstants.maxArmValue) {
      return ArmPhysicalConstants.maxArmValue;
    } else if (x < ArmPhysicalConstants.minArmValue) {
      return ArmPhysicalConstants.minArmValue;
    } else {
      return x;
    }
  }

  public void incrementPosition(double increment) {
    m_currentPosition += increment;

  }

  public void setPosition(double position) {
    m_currentPosition = position;
  }

  public double getTargetPosition() {
    m_currentPosition = clampValue(m_currentPosition);
    return m_currentPosition;
  }

  public void setArmPosition(double position) {
    m_fixedposition = true;
    // limit position to safe values
    // m_clampedPosition = clampValue(position);
    m_PIDController.setReference(position, ControlType.kSmartMotion);
    
  }
  public void setVelocityArm(double velocity){
    // System.out.println(velocity);
    m_PIDController.setReference(velocity, ControlType.kVelocity, ArmPIDConstants.defaultSlot);
  }
  public boolean isInFixedPositionMode(){
    return m_fixedposition;

  }
  
  public void canceledFixedPositionMode(){
    // System.out.println("-----------------CANCLED FIXED POS-----------------------");
    m_fixedposition = false;
  }

  public boolean isInPosition(double position){
    
    position = clampValue(position);
    if(getEncoderInches()>=position){
      
      // m_fixedposition = false;

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
    SmartDashboard.putBoolean("Fixed pos?", m_fixedposition);
    SmartDashboard.putNumber("ARM DEG", getEncoderDeg());
    if(m_lowerlimitswitch.isPressed()){
      m_encoder.setPosition(0);
    }
    // SmartDashboard.putBoolean("Upper limit Switch",
    // m_upperlimitswitch.isPressed());
    // SmartDashboard.putBoolean("Lower limit Switch",
    // m_lowerlimitswitch.isPressed());
    // This method will be called on ce per scheduler run
  }
}
