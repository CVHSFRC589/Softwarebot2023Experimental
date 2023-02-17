// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class WristSubsystem extends SubsystemBase {
  //declare wrist motor and encoder
  public WPI_TalonSRX m_wrist;
  public SensorCollection m_wristEncoder;
  public int m_wristRevolutions;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    // 
    //sets motor to motor id
    m_wrist = new WPI_TalonSRX(IDConstants.kWristPort);
    m_wrist.configFactoryDefault();
    m_wrist.setNeutralMode(NeutralMode.Brake);
    
    m_wristEncoder = m_wrist.getSensorCollection();

  }

  public void wristMove(DoubleSupplier joystick) {
    if(m_wristEncoder.getQuadraturePosition()==1){
      m_wristRevolutions++;
    }
    if (m_wristRevolutions<100) {
      m_wrist.set(joystick.getAsDouble());
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Quadrature Position", m_wristEncoder.getQuadraturePosition());
    SmartDashboard.putNumber("Wrist Position????", m_wrist.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }
}
