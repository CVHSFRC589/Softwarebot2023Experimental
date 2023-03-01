// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PhysicalConstants;

public class WristSubsystem extends SubsystemBase {
  // declare wrist motor and encoder
  public WPI_TalonSRX m_wrist;
  public TalonSRXConfiguration m_talonConfig;
  public SensorCollection m_wristEncoder;
  public double m_currentPosition;
  public Timer m_timer;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    // sets motor to motor id
    m_wrist = new WPI_TalonSRX(IDConstants.kWristPort);

    m_wrist.configFactoryDefault();

    m_wrist.set(TalonSRXControlMode.Position, 0);

    m_wrist.setNeutralMode(NeutralMode.Brake);
    m_talonConfig = new TalonSRXConfiguration();

    /* setup a basic closed loop */
    m_wrist.setNeutralMode(NeutralMode.Brake); // Netural Mode override
    m_wrist.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);// Sensor Type
    // Constants.PID_PRIMARY, // PID Index
    // Constants.kTimeoutMs); // Config Timeout

    /*
     * Ensure Sensor is in phase, else closed loop will not work.
     * Positive Sensor should match Motor Positive output (Green LED)
     */
    m_wrist.setSensorPhase(true);

    // m_wrist.configAllSettings();

    // /* Gains for Position Closed Loop servo */
    // m_wrist.config_kP(Constants.SLOT_0, Constants.kGains.kP,
    // Constants.kTimeoutMs);
    // m_wrist.config_kI(Constants.SLOT_0, Constants.kGains.kI,
    // Constants.kTimeoutMs);
    // m_wrist.config_kD(Constants.SLOT_0, Constants.kGains.kD,
    // Constants.kTimeoutMs);
    // m_wrist.config_kF(Constants.SLOT_0, Constants.kGains.kF,
    // Constants.kTimeoutMs);

    m_wristEncoder = m_wrist.getSensorCollection();
    // resetEncoders();

    m_currentPosition = getWristDeg();
  }

  public double getWristDeg() {
    return m_wristEncoder.getQuadraturePosition() * (1 / PhysicalConstants.WRIST_ENCODER_TO_DEG);

  }

  public void incrementPosition(double increment) {
    m_currentPosition += increment;

  }

  // public void resetEncoders() {
  // m_wristEncoder.setQuadraturePosition(0, 0);
  // SmartDashboard.putNumber("Wrist m_wristEncoder.getPulseWidthPosition()",
  // m_wristEncoder.getQuadraturePosition());

  // }

  public void wristSetPosition(double position) {

    m_wrist.set(ControlMode.Position, position);
  }

  public void wristSetVelocity(DoubleSupplier joystick) {

    // if (getWristDeg() <= 120 && getWristDeg() >= 0) {
    m_wrist.set(joystick.getAsDouble());
    // } else {
    // m_wrist.set(0);
    // }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Encoder", m_wristEncoder.getQuadraturePosition());
    SmartDashboard.putNumber("Wrist Degrees", getWristDeg());
    // This method will be called once per scheduler run
  }
}
