// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;

public class GripperSubsystem extends SubsystemBase {
private DoubleSolenoid m_gripper;
private Compressor m_compressor;
  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    m_gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    m_compressor.enableAnalog(0, 0);
  }

  public void open(){
    m_gripper.set(DoubleSolenoid.Value.kReverse);
  }

  public void close(){
    m_gripper.set(DoubleSolenoid.Value.kForward);
  }

  public void toggleGripperSolenoids(){
    if(getGripperValue().equals(DoubleSolenoid.Value.kForward))
      open();
    else
      close();
  }

  public DoubleSolenoid.Value getGripperValue(){
    return m_gripper.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
