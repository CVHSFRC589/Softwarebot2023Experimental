// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenGripper extends InstantCommand {
  private GripperSubsystem m_gripperSubsystem;
  private ArmSubsystem m_arm;
  public OpenGripper(GripperSubsystem gripper, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
    m_arm = arm;
    m_gripperSubsystem = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_arm.getPosition()<ArmPhysicalConstants.maxArmValue && m_arm.getPosition()>
    ArmPhysicalConstants.minArmValue+15){
      m_gripperSubsystem.open();
    }
    else {
      System.out.println("----------ARM NOT OPENING ------> OUT OF OPENING BOUNDS---------");
    }
  }
}
