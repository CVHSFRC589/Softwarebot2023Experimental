// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_ARM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFollowJoy extends CommandBase {
  private DoubleSupplier m_velocity;
  private DoubleSupplier m_velocitymodifier;
  private ArmSubsystem m_arm;

  /** Creates a new ArmFollowJoy. */
  public ArmFollowJoy(ArmSubsystem arm, DoubleSupplier velocity, DoubleSupplier velocitymodifier) {
    m_velocity = velocity;
    m_velocitymodifier = velocitymodifier;
    m_arm = arm;
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
    SmartDashboard.putNumber("Joystick Y velocity", m_velocity.getAsDouble());
    SmartDashboard.putNumber("Joystick slider velocity modifier", m_velocitymodifier.getAsDouble());

    double velocity = m_velocity.getAsDouble() * ((m_velocitymodifier.getAsDouble() + 1) / 2) * 750;

    m_arm.setVelocityArm(velocity);
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
