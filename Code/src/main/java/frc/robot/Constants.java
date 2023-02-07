// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double kStatic = 0.16935;
    
    public static final int kLeftMotorPort = 11;
    public static final int kRightMotorPort = 12;
    // Analog inputs
    public static final int kRangeFinderPort = 0;
    //pigeon!!!
    public static final int Pigeon2ID = 60;

   }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final Button buttonA = Button.kB;
    public static final Button buttonB = Button.kX;
    public static final Button buttonX = Button.kA;
    public static final Button buttonY = Button.kY;
  }

  public static final class PhysicalConstants {
    // Robot Measurement Constants
    public static final int DRIVE_WHEEL_RADIUS = 3;
    
    public static final double SOFIE_TURN_CIRCUM = Math.PI * 2 * 9.875;

    public static final double DRIVE_WHEEL_CIRCUM = 2 * Math.PI * DRIVE_WHEEL_RADIUS;
    public static final double DRIVE_GEAR_RATIO = 10.71;
  }
  
  public static final class PIDConstants {
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
  }
}
