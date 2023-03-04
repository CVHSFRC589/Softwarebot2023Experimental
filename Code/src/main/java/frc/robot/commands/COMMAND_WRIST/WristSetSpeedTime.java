// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_WRIST;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristSetSpeedTime extends CommandBase {
  private WristSubsystem m_wrist;
  private DoubleSupplier m_speed;
  private double m_time;
  private Timer m_timer = new Timer();
  private double m_startTime = 0.0;  

  /** Creates a new WristSetSpeed. */
  public WristSetSpeedTime(WristSubsystem wrist, DoubleSupplier speed, double time) {
    m_wrist = wrist;
    m_speed = speed;
    m_time = time;
    addRequirements(m_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_startTime = m_timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.wristSetVelocity(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.wristSetVelocity(() -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() >= m_startTime + m_time) {
      return true;
    } else {
      return false;
    }
  }
}
