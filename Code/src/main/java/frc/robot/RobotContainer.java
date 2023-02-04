// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveVoltage;
import frc.robot.commands.HalveDriveSpeed;
import frc.robot.commands.PIDLockInPlace;
import frc.robot.commands.QuarterDriveSpeed;
import frc.robot.commands.TurnDeg;
import frc.robot.commands.TurnDegGyro;
import frc.robot.commands.Auto_Pattern.ComplexAuto;
import frc.robot.commands.Auto_Pattern.Obstacle;
import frc.robot.commands.Auto_Pattern.RealComplexAuto;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
//  */
public class RobotContainer {
  //camera
  UsbCamera camera = CameraServer.startAutomaticCapture();
  // The robot's subsystems
  private DriveSubsystem m_robotDrive = new DriveSubsystem();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final Command m_complexAuto = new ComplexAuto(m_robotDrive);
  private final Command m_realcomplexAuto = new RealComplexAuto(m_robotDrive);
  private final Command m_Obstacle = new Obstacle(m_robotDrive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DefaultDrive(
            m_robotDrive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX()));
    
    m_chooser.setDefaultOption("Complex Auto", m_complexAuto);
    m_chooser.addOption("Real Complex Auto", m_realcomplexAuto);
    m_chooser.addOption("Obstacle", m_Obstacle);

    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putNumber("Motor voltage", DriveConstants.kStatic);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // While holding the shoulder button, drive at half speed
    //kX = button b
    //kA = button x
    //kB = button a
    //kY = button y
    // new JoystickButton(m_driverController, OIConstants.buttonY.value)
    //     .onTrue(new TurnDegGyro(90, .25, m_robotDrive));
    // new JoystickButton(m_driverController, OIConstants.buttonB.value)
    //     .onTrue(new DriveWithinDistance(40, .35, m_robotDrive));
    // new JoystickButton(m_driverController, OIConstants.buttonB.value)
    // .onTrue(new DriveVoltage(SmartDashboard.getNumber("Motor voltage",0), m_robotDrive));
    new JoystickButton(m_driverController, OIConstants.buttonB.value)
    .onTrue(new DriveVoltage(DriveConstants.kStatic+.1, m_robotDrive));
    new JoystickButton(m_driverController, OIConstants.buttonA.value)
        .onTrue(new TurnDeg(90, .5, m_robotDrive));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .toggleOnTrue(new HalveDriveSpeed(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .toggleOnTrue(new QuarterDriveSpeed(m_robotDrive));
    new JoystickButton(m_driverController, OIConstants.buttonX.value)
        .onTrue(new DriveDistance(60, 0.25, m_robotDrive));
    new JoystickButton(m_driverController, OIConstants.buttonY.value)
        .toggleOnTrue(new PIDLockInPlace(m_robotDrive,36));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
