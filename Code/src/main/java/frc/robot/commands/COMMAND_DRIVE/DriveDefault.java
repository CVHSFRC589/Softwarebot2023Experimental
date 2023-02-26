// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class DriveDefault extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_forward2;
  private final DoubleSupplier m_rotation;
  
  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public DriveDefault(DriveSubsystem subsystem, DoubleSupplier y1, DoubleSupplier y2, DoubleSupplier rotation) {
    m_drive = subsystem;
    m_forward = y1;
    m_forward2 = y2;
    m_rotation = rotation;
    
    addRequirements(m_drive);
  }

  

  @Override
  public void execute() {
    m_drive.drive(m_forward, m_forward2, m_rotation);
  }

  
}
