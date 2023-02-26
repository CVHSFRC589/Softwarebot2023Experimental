// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_MISC;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class Pause extends CommandBase {
  /** Creates a new Pause. */
  private Timer m_timer = new Timer();
  private double m_startTime = 0.0;
  private double m_delaySeconds = 0.0;

  public Pause(double delaySeconds) {
    m_delaySeconds = delaySeconds;
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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.get() >= m_startTime + m_delaySeconds){
      return true;
    }
    else{
      return false;
    }  
  }
}
