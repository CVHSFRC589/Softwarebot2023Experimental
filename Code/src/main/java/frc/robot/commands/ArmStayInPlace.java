// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmStayInPlace extends CommandBase {
  private ArmSubsystem m_arm;
  private DoubleSupplier m_position;
  // private DoubleSupplier m_speed;
  // private DoubleSupplier m_speedmodifier;


  
  /** Creates a new ArmMove. */
  public ArmStayInPlace(ArmSubsystem arm, DoubleSupplier position) {
    m_arm = arm;
    m_position = position;
    // m_speed = y;
    // m_speedmodifier = speedmodifier;
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
    // m_arm.incrementPosition(m_speed.getAsDouble()*m_speedmodifier.getAsDouble()*.1);
    
    m_arm.setArmPosition(m_position.getAsDouble());
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
