// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.Constants.PhysicalConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class DriveSubsystem extends SubsystemBase {

  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;
  CANSparkMax m_leftMotor2;
  CANSparkMax m_rightMotor2;

  private SparkMaxPIDController m_leftPIDController;
  private SparkMaxPIDController m_rightPIDController;
  private double processVariable;
  private double processVariableLeft;
  private double processVariableRight;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private static WPI_Pigeon2 m_pigeon2;

  private AnalogInput m_rangeFinder;

  private double percentLMotor;
  private double percentRMotor;
  private boolean m_PIDmode;
  private double m_maxoutput;

  private NetworkTable m_table;
  private NetworkTableEntry m_pattern;
  private NetworkTableEntry m_patternOver;

  private DifferentialDrive m_drive;
  private BooleanSupplier m_driveType;// true is arcade false is tank drive

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    m_leftMotor = new CANSparkMax(IDConstants.kLeftMotorPort, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(IDConstants.kRightMotorPort, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(IDConstants.kLeftMotorPort2, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(IDConstants.kRightMotorPort2, MotorType.kBrushless);
    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);
    m_PIDmode = false;
    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_rightMotor.setSmartCurrentLimit(PhysicalConstants.maxDriveAmps);
    m_leftMotor.setSmartCurrentLimit(PhysicalConstants.maxDriveAmps);

    // pigeon
    m_pigeon2 = new WPI_Pigeon2(IDConstants.Pigeon2ID);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();
    m_rangeFinder = new AnalogInput(IDConstants.kRangeFinderPort);
    m_leftPIDController = m_leftMotor.getPIDController();
    m_rightPIDController = m_rightMotor.getPIDController();
    resetEncoders();
    // motor settings
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);
    m_leftEncoder
        .setPositionConversionFactor(PhysicalConstants.DRIVE_WHEEL_CIRCUM / PhysicalConstants.DRIVE_GEAR_RATIO);
    m_rightEncoder
        .setPositionConversionFactor(PhysicalConstants.DRIVE_WHEEL_CIRCUM / PhysicalConstants.DRIVE_GEAR_RATIO);

    m_driveType = () -> false;
    // m_leftMotor.setOpenLoopRampRate(.35);
    // m_rightMotor.setOpenLoopRampRate(.35);

    // m_leftMotor.setClosedLoopRampRate(.5);
    // m_rightMotor.setClosedLoopRampRate(.5);
    // smart motion

    setPIDConstants();
    m_table = NetworkTableInstance.getDefault().getTable(LEDConstants.NETWORK_TABLE_NAME);
    m_patternOver = m_table.getEntry(LEDConstants.PATTERN_FINISHED_ENTRY_NAME);
    m_pattern = m_table.getEntry(LEDConstants.VISUAL_FEEDBACK_TABLE_ENTRY_NAME);

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // m_drive.arcadeDrive(fwd, rot, true);
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDrive(double y1, double y2) {
    // m_drive.arcadeDrive(fwd, rot, true);
    m_drive.tankDrive(y1, y2);
  }

  public void drive(DoubleSupplier y1, DoubleSupplier y2, DoubleSupplier rotation) {
    // if true run arcade drive
    m_drive.tankDrive(y1.getAsDouble(), y2.getAsDouble());
    // m_drive.arcadeDrive(y1.getAsDouble(), rotation.getAsDouble());
  }

  public void setDriveMode(boolean arcade) {
    m_driveType = () -> arcade;
  }

  public void setSafetyPID(boolean check) {
    m_drive.setSafetyEnabled(check);
  }

  public void smoothDrive(double y, double x) {
    double xbuffer = x;
    double ybuffer = y;
    double fwd;
    double rot;

    double leftRPM;
    double rightRPM;
    // Deadzone checking
    if (Math.abs(ybuffer) < .1) {
      ybuffer = 0;
    }

    if (Math.abs(xbuffer) < .1) {
      xbuffer = 0;
    }
    fwd = ybuffer;
    rot = xbuffer;
    percentRMotor = fwd + rot;
    percentLMotor = fwd - rot;

    if (Math.abs(percentLMotor) > 1) {
      if (percentLMotor > 0) {
        percentLMotor = 1;
      } else {
        percentLMotor = -1;
      }
    }

    if (Math.abs(percentRMotor) > 1) {
      if (percentRMotor > 0) {
        percentRMotor = 1;
      } else {
        percentRMotor = -1;
      }

    }

    leftRPM = percentLMotor * Constants.DrivePIDConstants.maxRPM;
    rightRPM = percentRMotor * Constants.DrivePIDConstants.maxRPM;
    m_rightPIDController.setReference(rightRPM, ControlType.kSmartVelocity,
        DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setReference(leftRPM, ControlType.kSmartVelocity,
        DrivePIDConstants.smartVelocitySlot);
    // m_leftPIDController.setReference(leftRPM, ControlType.kVelocity,2);
    // m_rightPIDController.setReference(rightRPM, ControlType.kVelocity,2);

  }
  public void smoothTankDrive(double y, double x) {
    double xbuffer = x;
    double ybuffer = y;
    double fwd;
    double rot;

    double leftRPM;
    double rightRPM;
    // Deadzone checking
    if (Math.abs(ybuffer) < .1) {
      ybuffer = 0;
    }

    if (Math.abs(xbuffer) < .1) {
      xbuffer = 0;
    }
    fwd = ybuffer;
    rot = xbuffer;
    percentRMotor = fwd + rot;
    percentLMotor = fwd - rot;

    if (Math.abs(percentLMotor) > 1) {
      if (percentLMotor > 0) {
        percentLMotor = 1;
      } else {
        percentLMotor = -1;
      }
    }

    if (Math.abs(percentRMotor) > 1) {
      if (percentRMotor > 0) {
        percentRMotor = 1;
      } else {
        percentRMotor = -1;
      }

    }

    leftRPM = percentLMotor * Constants.DrivePIDConstants.maxRPM;
    rightRPM = percentRMotor * Constants.DrivePIDConstants.maxRPM;
    m_rightPIDController.setReference(rightRPM, ControlType.kSmartVelocity,
        DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setReference(leftRPM, ControlType.kSmartVelocity,
        DrivePIDConstants.smartVelocitySlot);
    // m_leftPIDController.setReference(leftRPM, ControlType.kVelocity,2);
    // m_rightPIDController.setReference(rightRPM, ControlType.kVelocity,2);

  }

  public void setPIDMode() {
    resetMotors();
    // true means it is in pid mode, false is in normal drive
    m_PIDmode = true;
    m_pattern.setString("violet");
    m_patternOver.setString("not over");

  }

  public void cancelPIDMode() {
    m_PIDmode = false;
    resetMotors();
    m_patternOver.setString("over");
  }

  public void setVoltage(double voltage) {
    m_leftMotor.setVoltage(voltage);
    m_rightMotor.setVoltage(voltage);
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetMotors() {
    resetEncoders();
  }

  public double getLeftEncoderInches() {
    return m_leftEncoder.getPosition();// * PhysicalConstants.DRIVE_WHEEL_CIRCUM / PhysicalConstants.DRIVE_GEAR_RATIO;
  }

  public double getRightEncoderInches() {
    return m_rightEncoder.getPosition();// * PhysicalConstants.DRIVE_WHEEL_CIRCUM / PhysicalConstants.DRIVE_GEAR_RATIO;
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderInches() + getRightEncoderInches()) / 2;
  }

  public double getAbsAverageEncoderDistance() {
    return (Math.abs(getLeftEncoderInches()) + Math.abs(getRightEncoderInches())) / 2;
  }

  public void reset_gyro() {
    m_pigeon2.reset();
  }

  public double getPitch() {
    return m_pigeon2.getPitch();
  }

  public double getRoll() {
    return m_pigeon2.getRoll();
  }

  public double rollAdjust() {
    if (m_pigeon2.getRoll() > 2) {
      return m_pigeon2.getRoll() / 90;
    } else if (m_pigeon2.getRoll() < -2) {
      return m_pigeon2.getRoll() / 90;
    } else {
      return 0;
    }
  }

  public double pitchAdjust() {
    if (m_pigeon2.getPitch() > 2) {
      return m_pigeon2.getPitch() / -90;
    } else if (m_pigeon2.getPitch() < -2) {
      return m_pigeon2.getPitch() / -90;
    } else {
      return 0;
    }
  }

  public double pitchAdjustVelocity() {
    if (m_pigeon2.getPitch() > 3) {
      return m_pigeon2.getPitch() * -40;
    } else if (m_pigeon2.getPitch() < -3) {
      return m_pigeon2.getPitch() * -40;
    } else {
      return 0;
    }
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_maxoutput = maxOutput;
    m_drive.setMaxOutput(maxOutput);
  }

  // for finding the distance from the range finder
  public double getRangeFinderDistance() {
    double rawValue = m_rangeFinder.getValue();
    // double rangefinderVoltage = m_rangeFinder.getAverageVoltage();
    // double distanceInInches = (rangefinderVoltage * 65.4) - 7.2;
    // return distanceInInches;
    // voltage_scale_factor allows us to compensate for differences in supply
    // voltage.

    double voltage_scale_factor = 5;// 5/RobotController.getVoltage5V();

    double currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;

    return rawValue/7.73;

  }

  public void setPIDConstants() {
    // PID CONTROLER SMART MOTION CONSTANTS
    m_leftPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVel, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVel, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAcc, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErr,
        DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartMotionSlot);

    m_rightPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVel, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVel, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAcc, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErr,
        DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartMotionSlot);
    // PID CONSTANTS
    m_leftPIDController.setP(DrivePIDConstants.kP, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setI(DrivePIDConstants.kI, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setD(DrivePIDConstants.kD, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setIZone(DrivePIDConstants.kIz, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setFF(DrivePIDConstants.kFF, DrivePIDConstants.smartMotionSlot);
    // Spark manual says to do this via the desktop client
    // m_leftPIDController.setOutputRange(PIDConstants.kMinOutput,
    // PIDConstants.kMaxOutput);

    // set PID coefficients for right motor 2
    m_rightPIDController.setP(DrivePIDConstants.kP, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setI(DrivePIDConstants.kI, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setD(DrivePIDConstants.kD, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setIZone(DrivePIDConstants.kIz, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setFF(DrivePIDConstants.kFF, DrivePIDConstants.smartMotionSlot);
    // VELOCITY SLOTS

    // PID CONTROLER SMART MOTION CONSTANTS
    m_leftPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVelsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVelsv,
        DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAccsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErrsv,
        DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartVelocitySlot);

    m_rightPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVelsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVelsv,
        DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAccsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErrsv,
        DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartVelocitySlot);

    m_leftPIDController.setP(DrivePIDConstants.kPsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setI(DrivePIDConstants.kIsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setD(DrivePIDConstants.kDsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setIZone(DrivePIDConstants.kIzsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setFF(DrivePIDConstants.kFFsv, DrivePIDConstants.smartVelocitySlot);
    // Spark manual says to do this via the desktop client
    // m_leftPIDController.setOutputRange(PIDConstants.kMinOutput,
    // PIDConstants.kMaxOutput);

    // set PID coefficients for right motor 2
    m_rightPIDController.setP(DrivePIDConstants.kPsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setI(DrivePIDConstants.kIsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setD(DrivePIDConstants.kDsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setIZone(DrivePIDConstants.kIzsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setFF(DrivePIDConstants.kFFsv, DrivePIDConstants.smartVelocitySlot);

  }

  public void setVelocityLeftMotor(double velocity) {
    m_leftPIDController.setReference(velocity, CANSparkMax.ControlType.kSmartVelocity,
        DrivePIDConstants.smartVelocitySlot);
    processVariable = pitchAdjustVelocity();
    processVariableLeft = m_leftEncoder.getVelocity();

  }

  public void setVelocityRightMotor(double velocity) {
    m_rightPIDController.setReference(velocity, CANSparkMax.ControlType.kSmartVelocity,
        DrivePIDConstants.smartVelocitySlot);
    processVariable = pitchAdjustVelocity();
    processVariableRight = m_rightEncoder.getVelocity();
  }

  public void setPositionLeftMotor(double position) {
    m_leftPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion, DrivePIDConstants.smartMotionSlot);
    processVariableLeft = m_leftEncoder.getPosition();
  }

  public void setPositionRightMotor(double position) {
    m_rightPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion,
        DrivePIDConstants.smartMotionSlot);
    processVariableRight = m_rightEncoder.getPosition();
  }
  public double leftmotor1getMotorTemperature(){
    return m_leftMotor.getMotorTemperature();
  }
  public double rightmotor1getMotorTemperature(){
    return m_rightMotor.getMotorTemperature();
  }
  public double leftmotor2getMotorTemperature(){
    return m_leftMotor2.getMotorTemperature();
  }
  public double rightmotor2getMotorTemperature(){
    return m_rightMotor2.getMotorTemperature();
  }
  @Override
  public void periodic() {
    
    // double temperature = m_leftMotor.getMotorTemperature();

    
    // SmartDashboard.putNumber("Motor Temperature", temperature);
    SmartDashboard.putNumber("Left motor 1 temp", m_leftMotor.getMotorTemperature());
    SmartDashboard.putNumber("Left motor 2 temp", leftmotor2getMotorTemperature());
    SmartDashboard.putNumber("right motor 1 temp", rightmotor1getMotorTemperature());
    SmartDashboard.putNumber("right motor 2 temp", rightmotor2getMotorTemperature());

    SmartDashboard.putNumber("Encoder Abs Avg", getAbsAverageEncoderDistance());
    SmartDashboard.putNumber("Dist From Wall", getRangeFinderDistance());
    SmartDashboard.putNumber("DRIVE SPEED", m_maxoutput);
    SmartDashboard.putNumber("Encoder Position", getAverageEncoderDistance());
    SmartDashboard.putNumber("Encoder Ticks", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Process Variable", processVariable);

    // PIGEON
    SmartDashboard.putData(m_pigeon2);
    SmartDashboard.putNumber("Pigeon Pitch", m_pigeon2.getPitch());
    SmartDashboard.putNumber("Pigeon Roll", m_pigeon2.getRoll());
    // SmartDashboard.putNumber("Pigeon Yaw", m_pigeon2.getYaw());

    SmartDashboard.putNumber("Left PID Value", processVariableLeft);
    SmartDashboard.putNumber("Right PID Value", processVariableRight);
    SmartDashboard.putNumber("LMotor Percentage", percentLMotor);
    SmartDashboard.putNumber("RMotor Percentage", percentRMotor);
    SmartDashboard.putBoolean("LOCKED?", m_PIDmode);
  }
}
