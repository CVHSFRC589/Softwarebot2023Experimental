// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.Constants.PhysicalConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
public class DriveSubsystem extends SubsystemBase {
 // private AHRS navx;
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
  private boolean m_PIDmode;
  private static WPI_Pigeon2 m_pigeon2;
  
  private AnalogInput m_rangeFinder;

  private double percentLMotor;
  private double percentRMotor;
  
  private double Psv = 0.00004; 
  private double Isv = 0.000000000;
  private double Dsv = 0.00000000;
  private double Izsv = 0;
  private double FFsv = 0;
  private double MaxOutputsv = 1;
  private double MinOutputsv = -1;
  private double maxRPMsv = 5700;
  private double maxVelsv = 5700;
  private double minVelsv = 0;
  private double maxAccsv = 3500;
  private double allowedErrsv = 0.5;

  // The robot's drive
  private DifferentialDrive m_drive;
  private BooleanSupplier m_driveType;//true is arcade false is tank drive
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //
  //  navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

    m_leftMotor = new CANSparkMax(IDConstants.kLeftMotorPort, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(IDConstants.kRightMotorPort, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(IDConstants.kLeftMotorPort2, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(IDConstants.kRightMotorPort2, MotorType.kBrushless);
    
    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);

    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_rightMotor.setSmartCurrentLimit(PhysicalConstants.maxDriveAmps);
    m_leftMotor.setSmartCurrentLimit(PhysicalConstants.maxDriveAmps);

    //pigeon
    m_pigeon2 = new WPI_Pigeon2(IDConstants.Pigeon2ID);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();
    m_rangeFinder = new AnalogInput(IDConstants.kRangeFinderPort);
    m_PIDmode = false;
    m_leftPIDController = m_leftMotor.getPIDController();
    m_rightPIDController = m_rightMotor.getPIDController();
    resetEncoders();
    //motor settings
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(false);
    m_leftEncoder.setPositionConversionFactor(1.76);
    m_rightEncoder.setPositionConversionFactor(1.76);

    m_driveType = () -> false;
    // m_leftMotor.setOpenLoopRampRate(.35);
    // m_rightMotor.setOpenLoopRampRate(.35);

    // m_leftMotor.setClosedLoopRampRate(.5);
    // m_rightMotor.setClosedLoopRampRate(.5);
    //smart motion
    
    
    setPIDConstants();
    setSmartdashboard();

    

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

  public void drive(DoubleSupplier y1, DoubleSupplier y2, DoubleSupplier rotation){
    //if true run arcade drive
    // System.out.println("------------"+ m_driveType.getAsBoolean()+"----------");
    //m_drive.tankDrive(y1.getAsDouble(), y2.getAsDouble());
    m_drive.arcadeDrive(y1.getAsDouble(), rotation.getAsDouble());
    // if(m_driveType.getAsBoolean()){
      
    //   System.out.println("------------"+ y1.getAsDouble()+"----------");
    //   System.out.println("------------"+ y2.getAsDouble()+"----------");
    //   System.out.println("------------"+ rotation.getAsDouble()+"----------");
    // }
    // //else run tank!!! ---> default option is tank
    // else{
    //   m_drive.tankDrive(y1.getAsDouble(), y2.getAsDouble());
    // }
  }

  public void setDriveMode(boolean arcade){
    m_driveType = () -> arcade;
    System.out.println("--------------"+m_driveType+"------------");
  }

  public void cancelPIDMode() {
    m_PIDmode = false;
    resetMotors();
  }

  public void setSafetyPID(boolean check){
    m_drive.setSafetyEnabled(check);
  }

  public void setSmartdashboard(){
    SmartDashboard.putNumber("P Gain", Psv);
    SmartDashboard.putNumber("I Gain", Isv);
    SmartDashboard.putNumber("D Gain", Dsv);
    SmartDashboard.putNumber("I Zone", Izsv);
    SmartDashboard.putNumber("Feed Forward", FFsv);
    SmartDashboard.putNumber("Max Output", MaxOutputsv);
    SmartDashboard.putNumber("Min Output", MinOutputsv);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVelsv);
    SmartDashboard.putNumber("Min Velocity", minVelsv);
    SmartDashboard.putNumber("Max Acceleration", maxAccsv);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErrsv);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);
  }

  public void smoothDrive(double y, double x){
    double xbuffer = x;
    double ybuffer = y;
    double fwd;
    double rot;
    
    double leftRPM;
    double rightRPM;
    //Deadzone checking
    if(Math.abs(ybuffer)<.1){
      ybuffer = 0;
    }
    
    if(Math.abs(xbuffer)<.1){
      xbuffer = 0;
    }
    fwd = ybuffer;
    rot = xbuffer;
    percentRMotor = fwd+rot;
    percentLMotor = fwd-rot;
    
    if(Math.abs(percentLMotor)>1){
      if(percentLMotor>0){
        percentLMotor = 1;
      }
      else{
        percentLMotor = -1;
      }
    }

    if(Math.abs(percentRMotor)>1){
      if(percentRMotor>0){
        percentRMotor = 1;
      }
      else{
        percentRMotor = -1;
      }
      
    }

    leftRPM = percentLMotor*Constants.DrivePIDConstants.maxRPM;
    rightRPM = percentRMotor*Constants.DrivePIDConstants.maxRPM;
    m_rightPIDController.setReference(rightRPM, ControlType.kSmartVelocity, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setReference(leftRPM, ControlType.kSmartVelocity, DrivePIDConstants.smartVelocitySlot);
    // m_leftPIDController.setReference(leftRPM, ControlType.kVelocity,2);
    // m_rightPIDController.setReference(rightRPM, ControlType.kVelocity,2);

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
    m_leftPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVelsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVelsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAccsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErrsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartVelocitySlot);

    m_rightPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVelsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVelsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAccsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErrsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartVelocitySlot);


    m_leftPIDController.setP(DrivePIDConstants.kPsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setI(DrivePIDConstants.kIsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setD(DrivePIDConstants.kDsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setIZone(DrivePIDConstants.kIzsv, DrivePIDConstants.smartVelocitySlot);
    m_leftPIDController.setFF(DrivePIDConstants.kFFsv, DrivePIDConstants.smartVelocitySlot);
// Spark manual says to do this via the desktop client    m_leftPIDController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
    
    //set PID coefficients for right motor 2
    m_rightPIDController.setP(DrivePIDConstants.kPsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setI(DrivePIDConstants.kIsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setD(DrivePIDConstants.kDsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setIZone(DrivePIDConstants.kIzsv, DrivePIDConstants.smartVelocitySlot);
    m_rightPIDController.setFF(DrivePIDConstants.kFFsv, DrivePIDConstants.smartVelocitySlot);

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
  public void changePIDSVValues(){
   
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    
    if((p != Psv)) { m_leftPIDController.setP(p,2); m_rightPIDController.setP(p,2); Psv = p; }
    if((i != Isv)) { m_leftPIDController.setI(i,2); m_rightPIDController.setI(i,2); Isv = i; }
    if((d != Dsv)) { m_leftPIDController.setD(d,2); m_rightPIDController.setD(d,2); Dsv = d; }
    if((iz != Izsv)) { m_leftPIDController.setIZone(iz); m_rightPIDController.setIZone(iz); Izsv = iz; }
    if((ff != FFsv)) { m_leftPIDController.setFF(ff); m_rightPIDController.setFF(ff); FFsv = ff; }
    if((max != MaxOutputsv) || (min != MinOutputsv)) {  m_leftPIDController.setOutputRange(min, max); MinOutputsv = min; MaxOutputsv = max; }
    if((maxV != maxVelsv)) { m_leftPIDController.setSmartMotionMaxVelocity(maxV,2);m_rightPIDController.setSmartMotionMaxVelocity(maxV,2); maxVelsv = maxV; }
    if((minV != minVelsv)) { m_leftPIDController.setSmartMotionMinOutputVelocity(minV,2);m_rightPIDController.setSmartMotionMinOutputVelocity(minV,2); minVelsv = minV; }
    if((maxA != maxAccsv)) { m_leftPIDController.setSmartMotionMaxAccel(maxA,2);m_rightPIDController.setSmartMotionMaxAccel(maxA,2); maxAccsv = maxA; }
    if((allE != allowedErrsv)) { m_leftPIDController.setSmartMotionAllowedClosedLoopError(allE,2);m_rightPIDController.setSmartMotionAllowedClosedLoopError(allE,2); allowedErrsv = allE; }


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
      //PIGEON!!!!!!!!!!!!!!
      SmartDashboard.putData(m_pigeon2);
      SmartDashboard.putNumber("Pigeon Pitch", m_pigeon2.getPitch());
      SmartDashboard.putNumber("Pigeon Roll", m_pigeon2.getRoll());
      SmartDashboard.putNumber("Pigeon Rate", m_pigeon2.getRate());
      SmartDashboard.putNumber("Pigeon Temp", m_pigeon2.getTemp());
      SmartDashboard.putNumber("Pigeon Yaw", m_pigeon2.getYaw());
      SmartDashboard.putNumber("Pigeon Compass Heading", m_pigeon2.getCompassHeading()); 
      SmartDashboard.putNumber("Pigeon Handle", m_pigeon2.getHandle());
      SmartDashboard.putNumber("Pigeon Angle", m_pigeon2.getAngle());
      SmartDashboard.putNumber("Pigeon Field Strength", m_pigeon2.getCompassFieldStrength());
      SmartDashboard.putNumber("Pigeon Absolute Compass Heading", m_pigeon2.getAbsoluteCompassHeading());
      
      // SmartDashboard.putData(this);
      SmartDashboard.putNumber("Left PID Value", processVariableLeft);
      SmartDashboard.putNumber("Right PID Value", processVariableRight);
      SmartDashboard.putNumber("LMotor Percentage", percentLMotor);
      SmartDashboard.putNumber("RMotor Percentage", percentRMotor);
      changePIDSVValues();
    }
}
