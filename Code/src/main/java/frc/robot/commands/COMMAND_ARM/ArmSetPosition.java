// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_ARM;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetPosition extends CommandBase {
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
    // m_position = m_arm.clampValue(m_position);
    m_arm.setArmPosition(m_position);
  }

  @Override
  public boolean isFinished() {
    return !m_arm.isInFixedPositionMode();
    // return m_arm.isInPosition(m_position);
  }

}
