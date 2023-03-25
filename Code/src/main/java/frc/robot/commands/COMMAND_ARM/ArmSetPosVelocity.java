// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_ARM;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetPosVelocity extends CommandBase {
  /** Creates a new ArmSetPosVelocity. */
  private final ArmSubsystem m_arm;
  private final double m_degrees;

  public ArmSetPosVelocity(ArmSubsystem arm, double degrees) {
    m_arm = arm;
    m_degrees = degrees;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.retractPiston();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if ((m_arm.getEncoderDeg() <= m_degrees + 5 &&
    //     m_arm.getEncoderDeg() >= m_degrees - 5)) {//&&
    //     //m_arm.isPistonOpen()) {
    //       m_arm.setArmPosition(m_degrees);
    // } else {
      if (m_degrees < m_arm.getEncoderDeg()) {
        m_arm.setVelocityArm(-2000);
      } else {
        m_arm.setVelocityArm(2000);
      }
    }
    // }

    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setArmPosition(m_degrees);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    if ((m_arm.getEncoderDeg() <= m_degrees + .5 &&
        m_arm.getEncoderDeg() >= m_degrees - .5)) {//&&
        //m_arm.isPistonOpen()) {
      return true;
    } else {
      return false;
    }
  }
}
