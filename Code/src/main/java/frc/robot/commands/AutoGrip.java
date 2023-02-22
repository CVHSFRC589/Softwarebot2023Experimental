// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class AutoGrip extends CommandBase {
  /** Creates a new AutoGrip. */
  private ArmSubsystem m_arm;
  private GripperSubsystem m_grip;
  public AutoGrip(GripperSubsystem grip, ArmSubsystem arm) {
    m_grip = grip;
    m_arm = arm;
    addRequirements(grip);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.getTargetPosition()<ArmPhysicalConstants.minOpenGripThreshold){
      m_grip.close();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
