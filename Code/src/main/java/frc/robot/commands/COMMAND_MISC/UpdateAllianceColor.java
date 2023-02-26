// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_MISC;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.VisualFeedbackSubsystem;

public class UpdateAllianceColor extends InstantCommand {
  private VisualFeedbackSubsystem m_vfs;
  private NetworkTable m_table;
  private NetworkTableEntry m_patternOver;
  /** Creates a new SetVisualFeedbackMode. */
  public UpdateAllianceColor(VisualFeedbackSubsystem vfs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vfs = vfs;
    m_table = NetworkTableInstance.getDefault().getTable(LEDConstants.NETWORK_TABLE_NAME);
    m_patternOver = m_table.getEntry(LEDConstants.PATTERN_FINISHED_ENTRY_NAME);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vfs.setAllianceColor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_patternOver.setString("done");
  }
}
