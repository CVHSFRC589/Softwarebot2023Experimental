// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPIDConstants;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.Constants.IDConstants;
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
  private DoubleSolenoid m_armPiston;

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
    m_armPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, IDConstants.kPistonForward,
        IDConstants.kPistonReverse);
    m_lowerlimitswitch = m_motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_upperlimitswitch = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
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

    m_PIDController.setSmartMotionMaxVelocity(ArmPIDConstants.maxVelSM, ArmPIDConstants.smartMotionSlot);
    m_PIDController.setSmartMotionMinOutputVelocity(ArmPIDConstants.minVelSM, ArmPIDConstants.smartMotionSlot);
    m_PIDController.setSmartMotionMaxAccel(ArmPIDConstants.maxAccSM, ArmPIDConstants.smartMotionSlot);
    m_PIDController.setSmartMotionAllowedClosedLoopError(ArmPIDConstants.allowedErrSM, ArmPIDConstants.smartMotionSlot);
    m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, ArmPIDConstants.smartMotionSlot);

    m_PIDController.setP(ArmPIDConstants.kPSM, ArmPIDConstants.smartMotionSlot);
    m_PIDController.setI(ArmPIDConstants.kISM, ArmPIDConstants.smartMotionSlot);
    m_PIDController.setD(ArmPIDConstants.kDSM, ArmPIDConstants.smartMotionSlot);
    m_PIDController.setIZone(ArmPIDConstants.kIzSM, ArmPIDConstants.smartMotionSlot);
    m_PIDController.setFF(ArmPIDConstants.kFFSM, ArmPIDConstants.smartMotionSlot);


    m_upperlimitswitch.enableLimitSwitch(true);
    m_lowerlimitswitch.enableLimitSwitch(true);

  }
  // ==========================================SENSOR METHODS========================================== \\
  
  public double getEncoderDeg() {
    return m_encoder.getPosition() * 360;
  }

  public boolean isLowerLimitSwitchPressed() {
    return m_lowerlimitswitch.isPressed();
  }

  public boolean isUpperLimitSwitchPressed() {
    return m_upperlimitswitch.isPressed();
  }

  public void resetEncoders() {
    m_encoder.setPosition(0);
  }

  public double getEncoderInches() {
    return m_encoder.getPosition();
  }

  public boolean isInPosition(double position) {

    position = clampValue(position);
    if (getEncoderInches() >= position) {

      // m_fixedposition = false;

      return true;
    } else {
      return false;
    }

  }
  // ================================================================================================== \\

  // ==========================================PISTON METHODS========================================== \\
  
  public void toggleGripperSolenoids() {
    if (getPistonValue().equals(DoubleSolenoid.Value.kForward))
      openPiston();
    else
      closePiston();
  }

  public void openPiston() {
    m_armPiston.set(DoubleSolenoid.Value.kReverse);

  }

  public void closePiston() {
    m_armPiston.set(DoubleSolenoid.Value.kForward);

  }

  public DoubleSolenoid.Value getPistonValue() {
    return m_armPiston.get();
  }

  public boolean isPistonOpen() {
    if (m_armPiston.get() == DoubleSolenoid.Value.kForward) {
      return true;
    } else {
      return false;
    }
  }
  // ================================================================================================== \\

  
  
  // ==========================================ARM METHODS========================================== \\
  
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
    // m_clampedPosition = clampValue(position);SM
    m_PIDController.setReference(position / 360, ControlType.kSmartMotion, ArmPIDConstants.smartMotionSlot);

  }

  public void setVelocityArm(double velocity) {
    // System.out.println(velocity);
    m_PIDController.setReference(velocity, ControlType.kVelocity, ArmPIDConstants.defaultSlot);
  }

  public boolean isInFixedPositionMode() {
    return m_fixedposition;

  }

  public void canceledFixedPositionMode() {
    m_fixedposition = false;
  }

  // ================================================================================================== \\


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position", getEncoderInches());
    SmartDashboard.putNumber("Current Position", m_currentPosition);
    SmartDashboard.putNumber("Clamped Position", m_clampedPosition);
    SmartDashboard.putBoolean("Fixed pos?", m_fixedposition);
    SmartDashboard.putNumber("ARM DEG", getEncoderDeg());
    SmartDashboard.putNumber("ARM AMP", m_motor.getOutputCurrent());
    SmartDashboard.putBoolean("Upper limit Switch", m_upperlimitswitch.isPressed());
    SmartDashboard.putBoolean("Lower limit Switch", m_lowerlimitswitch.isPressed());
    //CHECK IF ARM IS ZEROED ---> SET ZERO
    if (m_lowerlimitswitch.isPressed()) {
      m_encoder.setPosition(0);
    }
    SmartDashboard.putData(m_armPiston);
    // This method will be called on ce per scheduler run
  }
}
