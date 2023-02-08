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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DrivePIDConstants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_PIDController;
  private RelativeEncoder m_encoder;
  private double m_clampedPosition;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_motor = new CANSparkMax(DriveConstants.kArmMotorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_PIDController = m_motor.getPIDController();
    m_clampedPosition = 0;
    // resetEncoders();
    
    m_PIDController.setSmartMotionMaxVelocity(ArmPIDConstants.maxVel, 0);
    m_PIDController.setSmartMotionMinOutputVelocity(ArmPIDConstants.minVel, 0);
    m_PIDController.setSmartMotionMaxAccel(ArmPIDConstants.maxAcc,0);
    m_PIDController.setSmartMotionAllowedClosedLoopError(ArmPIDConstants.allowedErr, 0);
    m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);

    m_PIDController.setP(ArmPIDConstants.kP,0);
    m_PIDController.setI(ArmPIDConstants.kI,0);
    m_PIDController.setD(ArmPIDConstants.kD,0);
    m_PIDController.setIZone(ArmPIDConstants.kIz,0);
    m_PIDController.setFF(ArmPIDConstants.kFF,0);


  }
  

  public void resetEncoders(){
    m_encoder.setPosition(0);
  }

  public double getEncoderInches() {
    return m_encoder.getPosition();
  }

  private double clampValue(double max, double min, double x){
    if (x>max){
      return max;
    }
    else if (x<min){
      return min;
    }
    else {
      return x;
    }
  }
  
  public void setArmPosition(double position){
    //limit position to safe values
    m_clampedPosition = clampValue(30, 5, position);

    m_PIDController.setReference(m_clampedPosition, ControlType.kSmartMotion);
  }
  public boolean isInPosition(){

    if(Math.abs(getEncoderInches())>=m_clampedPosition){
      return true;
    }
    else{
      return false;
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position", getEncoderInches());
    SmartDashboard.putNumber("Clamped Position", m_clampedPosition);
     
    // This method will be called once per scheduler run
  }
}
