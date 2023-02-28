// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_GRIP;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleGripper extends InstantCommand {
  private GripperSubsystem m_grip;
  private ArmSubsystem m_arm;
  public ToggleGripper(GripperSubsystem gripper, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
    m_arm = arm;
    m_grip = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(m_arm.getEncoderDeg()>25){
      m_grip.toggleGripperSolenoids();
    // }
    
    //if(m_arm.getTargetPosition()<ArmPhysicalConstants.maxArmValue && m_arm.getTargetPosition()>ArmPhysicalConstants.minArmValue+ArmPhysicalConstants.minOpenGripThreshold){
      // if(m_grip.isGripperOpen()){
      //   m_grip.close();
      // }
      // else{
      //   m_grip.open();
      // }
    //}
    //else {
    //   System.out.println("----------GRIPPER NOT OPENING ------> OUT OF OPENING BOUNDS---------");
    // }
  }
}
