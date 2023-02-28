// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class IDConstants {
    public static final double kStatic = 0.16935;// min amount of voltage???
    public static final int kGripperForward = 0;
    public static final int kGripperReverse = 1;
    public static final int kArmMotorPort = 50;// 12;//50
    public static final int VISUAL_FEEDBACK_MOTOR_PORT = 0;
    public static final int kWristPort = 40;//40

    public static final int kLeftMotorPort = 11;// 11
    public static final int kRightMotorPort = 12;// 12;
    
    public static final int kLeftMotorPort2 = 13;// 11
    public static final int kRightMotorPort2 = 14;// 12;
    // Analog inputs
    public static final int kRangeFinderPort = 0;
    // pigeon!!!
    public static final int Pigeon2ID = 60;

  }

  public static final class OIConstants {
    public static final int kDriverController1Port = 0;
    public static final int kDriverController2Port = 1;
    public static final int kOperatorControllerPort = 2;

    public static final Button buttonA = Button.kB;
    public static final Button buttonB = Button.kX;
    public static final Button buttonX = Button.kA;
    public static final Button buttonY = Button.kY;

    public static final int kYaxis = 1;
    public static final int kXaxis = 0;
    public static final int kZaxis = 2;
    public static final int kSlideraxis = 3;

    // LOGITECH XBOX CONTROLLER BINDINGS
    public static final int kButtonXXBOX = 1;
    public static final int kButtonAXBOX = 2;
    public static final int kButtonBXBOX = 3;
    public static final int kButtonYXBOX = 4;
    public static final int kButtonLeftBumperXBOX = 5;
    public static final int kButtonRightBumperXBOX = 6;
    public static final int kButtonLeftTriggerXBOX = 7;
    public static final int kButtonRightTriggerXBOX = 8;
    public static final int kButtonBackXBOX = 9;
    public static final int kButtonStartXBOX = 10;
    public static final int kButtonLeftStickDownXBOX = 11;
    public static final int kButtonRightStickDownXBOX = 12;
  }

  public static final class PhysicalConstants {
    // Robot Measurement Constants
    public static final double WRIST_ENCODER_TO_DEG = 5.7;
    public static final int maxDriveAmps = 35;
    public static final int DRIVE_WHEEL_RADIUS = 3;
    public static final double ARM_GEAR_RATIO = .01;//.01;
    public static final double SOFIE_TURN_CIRCUM = Math.PI * 2 * 9.875;

    public static final double DRIVE_WHEEL_CIRCUM = 2 * Math.PI * DRIVE_WHEEL_RADIUS;
    public static final double DRIVE_GEAR_RATIO = 10.71;
  }

  public static final class DrivePIDConstants {
    public static final int defaultSlot = 0;
    public static final int smartMotionSlot = 1;
    public static final int smartVelocitySlot = 2;

    public static final double kP = 0.00025;
    public static final double kI = 0.000000;
    public static final double kD = 0.0000000;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700;
    public static final double maxVel = 3000;
    public static final double minVel = 0;
    public static final double maxAcc = 4000;
    public static final double allowedErr = 1;

    public static final double kPsv = 0.00025;
    public static final double kIsv = 0.000000000;
    public static final double kDsv = 0.00000000;
    public static final double kIzsv = 0;
    public static final double kFFsv = 0;
    public static final double kMaxOutputsv = 1;
    public static final double kMinOutputsv = -1;
    public static final double maxRPMsv = 5700;
    public static final double maxVelsv = 5700;
    public static final double minVelsv = 0;
    public static final double maxAccsv = 3500;
    public static final double allowedErrsv = 0;
  }

  public static final class ArmPIDConstants {
    public static final int defaultSlot = 0;
    public static final int smartMotionSlot = 1;
    public static final int smartVelocitySlot = 2;

    public static final double kP = 0.0003;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 1000;
    public static final double maxVel = 4500;
    public static final double minVel = 0;
    public static final double maxAcc = 2000;
    public static final double allowedErr = 0.0056;

    public static final double kPSM = 0.0012;
    public static final double kDSM = 0;
    public static final double kISM = 0;
    public static final double kIzSM = 0;
    public static final double kFFSM = 0;
    public static final double kMaxOutputSM = 1;
    public static final double kMinOutputSM = -1;
    public static final double maxRPMSM = 2000;
    public static final double maxVelSM = 4000;
    public static final double minVelSM = 0;
    public static final double maxAccSM = 4000;
    public static final double allowedErrSM = 0.0056;
  }

  public static final class ArmPhysicalConstants {
    public static final double level2cone = 80;
    public static final double level2cube = 80;
    public static final double levelportal = 15;
    public static final double level1conecube = 30;
    public static final double levelfloor = 30;

    public static final int maxArmAmp = 35;
    public static final double maxArmValue = 31;
    public static final double minArmValue = -1;
    public static final double minOpenGripDeg = 35;

  }

  public static final class LEDConstants {
    public static final String NETWORK_TABLE_NAME = "Visual Feedback";
    public static final String VISUAL_FEEDBACK_TABLE_ENTRY_NAME = "VF Pattern";
    public static final String PATTERN_FINISHED_ENTRY_NAME = "VF Pattern done";
  }

}
