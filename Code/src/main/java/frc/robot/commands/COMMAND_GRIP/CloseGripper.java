// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_GRIP;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CloseGripper extends InstantCommand {
  private GripperSubsystem m_grip;

  public CloseGripper(GripperSubsystem gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
    m_grip = gripper;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_grip.close();
  }

}
