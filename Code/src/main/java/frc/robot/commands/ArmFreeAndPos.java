// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmFreeAndPos extends CommandBase {
  /** Creates a new ArmFreeAndPos. */
  private ArmSubsystem m_arm;
  private DoubleSupplier m_speed;
  private DoubleSupplier m_throttle;
  private DoubleSupplier m_position;

  public ArmFreeAndPos(ArmSubsystem arm, DoubleSupplier speed, DoubleSupplier throttle, DoubleSupplier position) {
    m_arm = arm;
    m_speed = speed;
    m_throttle = throttle;
    m_position = position;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.armFreeAndPosition(m_speed, m_throttle, m_position.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
