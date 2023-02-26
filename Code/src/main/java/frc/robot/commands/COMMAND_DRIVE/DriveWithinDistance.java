// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithinDistance extends CommandBase {
  /** Creates a new getWithin. */
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;

  public DriveWithinDistance(double inches, double speed, DriveSubsystem drive) {
    m_drive = drive;
    m_speed = speed;
    m_distance = inches;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    if(m_drive.getRangeFinderDistance()>m_distance)
    {
        m_drive.arcadeDrive(m_speed,0);
    }

    else if(m_drive.getRangeFinderDistance()<m_distance)
    {
        m_drive.arcadeDrive(-m_speed,0);
    }
  
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if(m_distance==m_drive.getRangeFinderDistance());
      return m_distance<=(m_drive.getRangeFinderDistance()+0.75) && m_distance>=(m_drive.getRangeFinderDistance()-0.75);
  }
}
