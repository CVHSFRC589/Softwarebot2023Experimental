// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PigeonBalance extends CommandBase {
  /** Creates a new PigeonBalance. */
  private DriveSubsystem m_drive;
  private double m_currentPosition;
  private Timer m_timer;
  private boolean m_atRest;

  public PigeonBalance(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_currentPosition = 0;
    m_timer = new Timer();
    m_atRest = false;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_atRest = false;
    m_drive.setPIDMode();
    m_timer.reset();
    m_timer.stop();
    m_drive.setSafetyPID(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_timer.reset();
    m_drive.setPositionLeftMotor(m_currentPosition += m_drive.pitchAdjust());
    m_drive.setPositionRightMotor(m_currentPosition += m_drive.pitchAdjust());

    if (m_drive.pitchAdjust() == 0 && !(m_atRest)) {
      m_timer.start();
      m_atRest = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.cancelPIDMode();
    m_timer.reset();
    m_drive.setSafetyPID(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_atRest && m_timer.get() > 3) {
      return true;
    }

    else {
      return false;
    }
  }

}