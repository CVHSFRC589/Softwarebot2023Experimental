// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.PhysicalConstants;

public class DriveSubsystem extends SubsystemBase {
  private AHRS navx;

  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;

  private SparkMaxPIDController m_leftPIDController;
  private SparkMaxPIDController m_rightPIDController;
  private double processVariable;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private boolean m_PIDmode;

  private AnalogInput m_rangeFinder;
  


  // The robot's drive
  private DifferentialDrive m_drive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //
    navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

    m_leftMotor = new CANSparkMax(DriveConstants.kLeftMotorPort, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(DriveConstants.kRightMotorPort, MotorType.kBrushless);
    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();
    m_rangeFinder = new AnalogInput(DriveConstants.kRangeFinderPort);
    m_PIDmode = false;
    m_leftPIDController = m_leftMotor.getPIDController();
    m_rightPIDController = m_rightMotor.getPIDController();
    resetMotors();
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void cancelPIDMode() {
    m_PIDmode = false;
    resetMotors();
  }

  public void setPIDMode() {
    resetMotors();
    //true means it is in pid mode, false is in normal drive
    m_PIDmode = true;

    // set PID coefficients
    m_leftPIDController.setP(PIDConstants.kP);
    m_leftPIDController.setI(PIDConstants.kI);
    m_leftPIDController.setD(PIDConstants.kD);
    m_leftPIDController.setIZone(PIDConstants.kIz);
    m_leftPIDController.setFF(PIDConstants.kFF);
// Spark manual says to do this via the desktop client    m_leftPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
 
    //set PID coefficients for right motor 2
    m_rightPIDController.setP(PIDConstants.kP);
    m_rightPIDController.setI(PIDConstants.kI);
    m_rightPIDController.setD(PIDConstants.kD);
    m_rightPIDController.setIZone(PIDConstants.kIz);
    m_rightPIDController.setFF(PIDConstants.kFF);
// Spark manual says to do this via the desktop client    m_rightPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
  }

  public void setVelocityLeftMotor(double velocity) {
    m_leftPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    processVariable = m_leftEncoder.getVelocity();
    
  }

  public void setVelocityRightMotor(double velocity) {
    m_rightPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    processVariable = m_rightEncoder.getVelocity();
  }

  public void setPositionLeftMotor(double position) {
    m_leftPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
    processVariable = m_leftEncoder.getPosition();
  }

  public void setPositionRightMotor(double position) {
    m_rightPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
    processVariable = m_rightEncoder.getPosition();
  }


  public void setVoltage(double voltage){
    m_leftMotor.setVoltage(voltage);
    m_rightMotor.setVoltage(voltage);
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetMotors(){
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    resetEncoders();
    
    //Our Specific settings
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);
    m_leftEncoder.setPositionConversionFactor(1);
    m_rightEncoder.setPositionConversionFactor(1);
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftEncoderInches() {
    return m_leftEncoder.getPosition() * PhysicalConstants.DRIVE_WHEEL_CIRCUM / PhysicalConstants.DRIVE_GEAR_RATIO;
  }

  public double getRightEncoderInches() {
    return m_rightEncoder.getPosition() * PhysicalConstants.DRIVE_WHEEL_CIRCUM / PhysicalConstants.DRIVE_GEAR_RATIO;
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderInches() + getRightEncoderInches()) / 2;
  }
  public double getAbsAverageEncoderDistance() {
    return (Math.abs(getLeftEncoderInches()) + Math.abs(getRightEncoderInches())) / 2;
  }
  public void reset_gyro(){
    navx.reset();
  }
  public double get_current_heading(){

    return navx.getAngle();
  } 

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
   // for finding the distance from the range finder
  public double getRangeFinderDistance() {
    double rawValue = m_rangeFinder.getValue();
    //  double rangefinderVoltage = m_rangeFinder.getAverageVoltage();
    //  double distanceInInches = (rangefinderVoltage * 65.4) - 7.2;
    //  return distanceInInches;
    //voltage_scale_factor allows us to compensate for differences in supply voltage.

    double voltage_scale_factor = 1;//5/RobotController.getVoltage5V();
      
    double currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;

    return currentDistanceInches;
    
  }
 
  @Override
    public void periodic() {
      SmartDashboard.putNumber("Encoder Abs Avg", getAbsAverageEncoderDistance());
      SmartDashboard.putNumber("Dist From Wall", getRangeFinderDistance());
      SmartDashboard.putData(navx);
      SmartDashboard.putNumber("Navx Pitch",navx.getPitch());
      // SmartDashboard.putNumber("Encoder Abs Avg", getAbsAverageEncoderDistance());
      SmartDashboard.putNumber("Encoder Position", getAverageEncoderDistance());
      SmartDashboard.putNumber("Encoder Ticks", m_leftEncoder.getPosition());//log();
      SmartDashboard.putNumber("Process Variable", processVariable);
    }
}
