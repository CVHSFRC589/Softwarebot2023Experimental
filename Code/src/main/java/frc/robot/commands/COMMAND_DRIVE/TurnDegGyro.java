// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

 

public class TurnDegGyro extends CommandBase {
  /** Creates a new TurnDegGyro. */
  private final DriveSubsystem m_drive;
  private final double m_degrees;
  private double m_speed;

  public TurnDegGyro(double degrees, double speed, DriveSubsystem drive) {
    m_drive = drive;
    m_speed = speed*-1;
    m_degrees = degrees;
    m_drive.reset_gyro();
   addRequirements(drive);
    }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.reset_gyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_speed);
    SmartDashboard.putNumber("m_speed(GYRO)", m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.get_current_heading() >= m_degrees;
  }
}
