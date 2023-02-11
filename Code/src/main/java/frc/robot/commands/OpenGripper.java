// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenGripper extends InstantCommand {
  private GripperSubsystem m_gripperSubsystem;
  public OpenGripper(GripperSubsystem gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
    m_gripperSubsystem = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripperSubsystem.open();
  }
}
