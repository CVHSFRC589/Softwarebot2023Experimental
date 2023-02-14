// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class WristSubsystem extends SubsystemBase {
  public WPI_TalonSRX m_wrist;
  public SensorCollection m_wristEncoder;
  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    m_wrist = new WPI_TalonSRX(IDConstants.kWristPort);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
