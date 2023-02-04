// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PIDLockInPlace extends CommandBase {
  /** Creates a new PIDDrive. */
  private DriveSubsystem m_drive;
  private double m_distance;
  public PIDLockInPlace(DriveSubsystem drive, double distance) {
    m_drive = drive;
    m_distance = distance;
    addRequirements(m_drive);    
  }

  // Call\ed when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setPIDMode();
    m_drive.setSafetyPID(false); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.setPositionLeftMotor(m_distance);
    m_drive.setPositionRightMotor(m_distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.cancelPIDMode();
    m_drive.setSafetyPID(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
