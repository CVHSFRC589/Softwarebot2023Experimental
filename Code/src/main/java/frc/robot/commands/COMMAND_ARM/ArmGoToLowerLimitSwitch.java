// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_ARM;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmGoToLowerLimitSwitch extends CommandBase {
  /** Creates a new ArmGoToLowerLimitSwitch. */
  private ArmSubsystem m_arm;

  public ArmGoToLowerLimitSwitch(ArmSubsystem arm) {
    addRequirements(arm);
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setVelocityArm(-589);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setVelocityArm(-589);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.resetEncoders();
    m_arm.setVelocityArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return m_arm.isLowerLimitSwitchPressed();
  }
}
