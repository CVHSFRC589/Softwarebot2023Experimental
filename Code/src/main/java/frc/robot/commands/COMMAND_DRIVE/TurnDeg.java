// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDeg extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_degrees;
  private double m_speed;

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed  The speed at which the robot will drive
   * @param drive  The drive subsystem on which this command will run
   */
  public TurnDeg(double degrees, double speed, DriveSubsystem drive) {
    m_degrees = degrees;
    m_speed = speed * -1;
    m_drive = drive;
    m_drive.resetEncoders();
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();
    /*
     * if (m_degrees<0){
     * m_speed = m_speed*-1;
     * }
     */
    // m_drive.arcadeDrive(0, m_speed);
  }

  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_speed);
    SmartDashboard.putNumber("m_speed", m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    // m_drive.resetEncoders();
  }

  @Override
  public boolean isFinished() {
    return m_drive.getAbsAverageEncoderDistance() >= (PhysicalConstants.SOFIE_TURN_CIRCUM) * (m_degrees / 360.0);
  }
}
