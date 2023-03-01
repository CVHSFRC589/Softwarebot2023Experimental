// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.PatternMap;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.LEDConstants;

public class VisualFeedbackSubsystem extends SubsystemBase {
  /** Creates a new VisualFeedbackSubsystem. */
  private final PWMSparkMax m_visualFeedbackDevice = new PWMSparkMax(IDConstants.VISUAL_FEEDBACK_MOTOR_PORT);
  private String defaultColor = "violet";
  private NetworkTable m_table;
  private NetworkTableEntry m_pattern;
  private NetworkTableEntry m_patternOver;
  private DriverStation.Alliance m_alliance;
  private PatternMap m_patternMap;
  private boolean m_override;

  public VisualFeedbackSubsystem() {
    m_table = NetworkTableInstance.getDefault().getTable(LEDConstants.NETWORK_TABLE_NAME);
    m_pattern = m_table.getEntry(LEDConstants.VISUAL_FEEDBACK_TABLE_ENTRY_NAME);
    m_patternOver = m_table.getEntry(LEDConstants.PATTERN_FINISHED_ENTRY_NAME);
    m_patternMap = new PatternMap();
  }

  public void setAllianceColor() {
    m_alliance = DriverStation.getAlliance();
    if (m_alliance.equals(DriverStation.Alliance.Red)) {
      defaultColor = "red";
    } else if (m_alliance.equals(DriverStation.Alliance.Blue)) {
      defaultColor = "blue";
    }
  }

  public void toggleOverride() {
    m_override = !m_override;
  }

  private void giveVisualFeedback(String color) {
    m_visualFeedbackDevice.set(m_patternMap.getPattern(color));
  }

  private boolean allDone() {
    if (m_patternOver.getString("").equals("done")) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_override) {
      m_pattern.setString("black");
    } else if (allDone()) {
      m_pattern.setString(defaultColor);
    }
    giveVisualFeedback(m_table.getEntry(LEDConstants.VISUAL_FEEDBACK_TABLE_ENTRY_NAME).getString(defaultColor));
  }
}
