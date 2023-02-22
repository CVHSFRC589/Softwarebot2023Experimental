// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;

public class GripperSubsystem extends SubsystemBase {
private double m_currentPosition;
private DoubleSolenoid m_gripper;
private Compressor m_compressor;
private NetworkTable m_table;
private NetworkTableEntry m_pattern;
private NetworkTableEntry m_patternOver;

  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    m_gripper = new DoubleSolenoid(PneumaticsModuleType.REVPH, IDConstants.kGripperForward, IDConstants.kGripperReverse); //new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    m_compressor.enableAnalog(0, 0);
    
    m_table = NetworkTableInstance.getDefault().getTable(LEDConstants.NETWORK_TABLE_NAME);
    m_patternOver = m_table.getEntry(LEDConstants.PATTERN_FINISHED_ENTRY_NAME);
    m_pattern = m_table.getEntry(LEDConstants.VISUAL_FEEDBACK_TABLE_ENTRY_NAME);
  }
  public void incrementPosition(double increment){
    m_currentPosition += increment;
    
  }
  public void open(){
    m_gripper.set(DoubleSolenoid.Value.kReverse);
    m_pattern.setString("rainbow party palette");
    m_patternOver.setString("not over");
  }

  public void close(){
    m_gripper.set(DoubleSolenoid.Value.kForward);
    m_pattern.setString("bpm forest palette");
    m_patternOver.setString("not over");
  }

  public void toggleGripperSolenoids(){
    if(getGripperValue().equals(DoubleSolenoid.Value.kForward))
      open();
    else
      close();
  }

  public boolean isGripperOpen(){
    if(m_gripper.get()==DoubleSolenoid.Value.kForward){
      return true;
    }
    else{
      return false;
    }
  }


  public DoubleSolenoid.Value getGripperValue(){
    return m_gripper.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(m_compressor);
    SmartDashboard.putData(m_gripper);
    // This method will be called once per scheduler run
  }
}
