// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.Constants.PhysicalConstants;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
public class DriveSubsystem extends SubsystemBase {
 // private AHRS navx;
  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;

  private SparkMaxPIDController m_leftPIDController;
  private SparkMaxPIDController m_rightPIDController;
  private double processVariable;
  private double processVariableLeft;
  private double processVariableRight;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private boolean m_PIDmode;
  private static WPI_Pigeon2 m_pigeon2;
  
  private AnalogInput m_rangeFinder;
  


  // The robot's drive
  private DifferentialDrive m_drive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //
  //  navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

    m_leftMotor = new CANSparkMax(DriveConstants.kLeftMotorPort, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(DriveConstants.kRightMotorPort, MotorType.kBrushless);
    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_rightMotor.setSmartCurrentLimit(PhysicalConstants.maxDriveAmps);
    m_leftMotor.setSmartCurrentLimit(PhysicalConstants.maxDriveAmps);

    //pigeon
    m_pigeon2 = new WPI_Pigeon2(DriveConstants.Pigeon2ID);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();
    m_rangeFinder = new AnalogInput(DriveConstants.kRangeFinderPort);
    m_PIDmode = false;
    m_leftPIDController = m_leftMotor.getPIDController();
    m_rightPIDController = m_rightMotor.getPIDController();
    resetEncoders();
    //motor settings
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);
    m_leftEncoder.setPositionConversionFactor(1.76);
    m_rightEncoder.setPositionConversionFactor(1.76);
    //smart motion
   
    
    setPIDConstants();

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot, true);
  }

  public void cancelPIDMode() {
    m_PIDmode = false;
    resetMotors();
  }

  public void setSafetyPID(boolean check){
    m_drive.setSafetyEnabled(check);
  }

  public void setPIDMode() {
    resetMotors();
    //true means it is in pid mode, false is in normal drive
    m_PIDmode = true;

    // set PID coefficients
//     m_leftPIDController.setP(PIDConstants.kP, PIDConstants.smartMotionSlot);
//     m_leftPIDController.setI(PIDConstants.kI, PIDConstants.smartMotionSlot);
//     m_leftPIDController.setD(PIDConstants.kD, PIDConstants.smartMotionSlot);
//     m_leftPIDController.setIZone(PIDConstants.kIz, PIDConstants.smartMotionSlot);
//     m_leftPIDController.setFF(PIDConstants.kFF, PIDConstants.smartMotionSlot);
// // Spark manual says to do this via the desktop client    m_leftPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
    
//     //set PID coefficients for right motor 2
//     m_rightPIDController.setP(PIDConstants.kP, PIDConstants.smartMotionSlot);
//     m_rightPIDController.setI(PIDConstants.kI, PIDConstants.smartMotionSlot);
//     m_rightPIDController.setD(PIDConstants.kD, PIDConstants.smartMotionSlot);
//     m_rightPIDController.setIZone(PIDConstants.kIz, PIDConstants.smartMotionSlot);
//     m_rightPIDController.setFF(PIDConstants.kFF, PIDConstants.smartMotionSlot);
// // Spark manual says to do this via the desktop client    m_rightPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
  }

  public void setPIDConstants(){
    //PID CONTROLER SMART MOTION CONSTANTS
    m_leftPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVel, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVel, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAcc, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErr, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartMotionSlot);

    m_rightPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVel, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVel, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAcc, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErr, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartMotionSlot);
  //PID CONSTANTS
    m_leftPIDController.setP(DrivePIDConstants.kP, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setI(DrivePIDConstants.kI, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setD(DrivePIDConstants.kD, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setIZone(DrivePIDConstants.kIz, DrivePIDConstants.smartMotionSlot);
    m_leftPIDController.setFF(DrivePIDConstants.kFF, DrivePIDConstants.smartMotionSlot);
// Spark manual says to do this via the desktop client    m_leftPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
    
    //set PID coefficients for right motor 2
    m_rightPIDController.setP(DrivePIDConstants.kP, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setI(DrivePIDConstants.kI, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setD(DrivePIDConstants.kD, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setIZone(DrivePIDConstants.kIz, DrivePIDConstants.smartMotionSlot);
    m_rightPIDController.setFF(DrivePIDConstants.kFF, DrivePIDConstants.smartMotionSlot);
    //VELOCITY SLOTS

 //PID CONTROLER SMART MOTION CONSTANTS
    m_leftPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVel, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVel, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAcc, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErr, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartVelocitySlot);

    m_rightPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVel, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVel, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAcc, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErr, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartVelocitySlot);


    m_leftPIDController.setP(DrivePIDConstants.kP, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setI(DrivePIDConstants.kI, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setD(DrivePIDConstants.kD, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setIZone(DrivePIDConstants.kIz, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setFF(DrivePIDConstants.kFF, DrivePIDConstants.smartVelocitySlot);
// Spark manual says to do this via the desktop client    m_leftPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
    
    //set PID coefficients for right motor 2
    m_rightPIDController.setP(DrivePIDConstants.kP, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setI(DrivePIDConstants.kI, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setD(DrivePIDConstants.kD, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setIZone(DrivePIDConstants.kIz, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setFF(DrivePIDConstants.kFF, DrivePIDConstants.smartVelocitySlot);

  }
  public void setVelocityLeftMotor(double velocity) {
    m_leftPIDController.setReference(velocity, CANSparkMax.ControlType.kSmartVelocity, DrivePIDConstants.smartVelocitySlot);
    processVariable = pitchAdjustVelocity();
    processVariableLeft = m_leftEncoder.getVelocity();
    
  }

  public void setVelocityRightMotor(double velocity) {
    m_rightPIDController.setReference(velocity, CANSparkMax.ControlType.kSmartVelocity, DrivePIDConstants.smartVelocitySlot);
    processVariable = pitchAdjustVelocity();
    processVariableRight = m_rightEncoder.getVelocity();
  }

  public void setPositionLeftMotor(double position) {
    m_leftPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion, DrivePIDConstants.smartMotionSlot);
    processVariableLeft = m_leftEncoder.getPosition();
  }

  public void setPositionRightMotor(double position) {
    m_rightPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion, DrivePIDConstants.smartMotionSlot);
    processVariableRight = m_rightEncoder.getPosition();
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
    //m_leftMotor.restoreFactoryDefaults();
    //m_rightMotor.restoreFactoryDefaults();
    resetEncoders();
    
    //Our Specific settings
   
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
  public void reset_gyro(){
   // navx.reset();
   m_pigeon2.reset();
  }
  public double get_current_heading(){
    return 0;
   // return navx.getAngle();
  }
  public double getPitch(){
    return m_pigeon2.getPitch();
  } 

  public double getRoll(){
    return m_pigeon2.getRoll();
  }

  public double rollAdjust(){
    if(m_pigeon2.getRoll() > 2){
      return m_pigeon2.getRoll() / 90;
    }
    else if(m_pigeon2.getRoll() < -2){
      return m_pigeon2.getRoll() / 90;
    }
    else{
      return 0;
    }
  }

  public double pitchAdjust() {
    if(m_pigeon2.getPitch() > 2){
      return m_pigeon2.getPitch() / -90;
    }
    else if(m_pigeon2.getPitch() < -2){
      return m_pigeon2.getPitch() / -90;
    }
    else{
      return 0;
    }
  }

  public double pitchAdjustVelocity() {
    if(m_pigeon2.getPitch() > 3){
      return m_pigeon2.getPitch() * -100;
    }
    else if(m_pigeon2.getPitch() < -3){
      return m_pigeon2.getPitch() * -100;
    }
    else{
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
      //SmartDashboard.putData(navx);
      //SmartDashboard.putNumber("Navx Pitch",navx.getPitch());
      // SmartDashboard.putNumber("Encoder Abs Avg", getAbsAverageEncoderDistance());
      SmartDashboard.putNumber("Encoder Position", getAverageEncoderDistance());
      SmartDashboard.putNumber("Encoder Ticks", m_leftEncoder.getPosition());//log();
      SmartDashboard.putNumber("Process Variable", processVariable);
      SmartDashboard.putData(m_pigeon2);
      SmartDashboard.putNumber("Pigeon Pitch", m_pigeon2.getPitch());
      SmartDashboard.putNumber("Pigeon Roll", m_pigeon2.getRoll());
      // SmartDashboard.putData(this);
      SmartDashboard.putNumber("Left PID Value", processVariableLeft);
      SmartDashboard.putNumber("Right PID Value", processVariableRight);
    }
}
