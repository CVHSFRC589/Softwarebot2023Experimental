// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSelectSpeed extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_speed;

  public DriveSelectSpeed(DriveSubsystem drive, double speed) {
    m_speed = speed;
    m_drive = drive;
  }

  @Override
  public void initialize() {
    m_drive.setMaxOutput(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setMaxOutput(1);
  }
}
