// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetPosition extends InstantCommand {
  private ArmSubsystem m_arm;
  private double m_position;
  /** Creates a new ArmSetPosition. */
  public ArmSetPosition(ArmSubsystem arm, double position) {
    m_position = position;
    m_arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setPosition(m_position);
  }
}
