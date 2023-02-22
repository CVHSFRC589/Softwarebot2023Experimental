// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.ArmFreeAndPos;
import frc.robot.commands.ArmSetPosition;
import frc.robot.commands.AutoGrip;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveAndBalance;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveVoltage;
import frc.robot.commands.HalveDriveSpeed;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.PIDLockInPlace;
import frc.robot.commands.PigeonBalance;
import frc.robot.commands.PigeonBalanceSmartVelocity;
import frc.robot.commands.QuarterDriveSpeed;
import frc.robot.commands.SetDriveMode;
import frc.robot.commands.SmoothDrive;
import frc.robot.commands.ToggleGripper;
import frc.robot.commands.TurnDeg;
import frc.robot.commands.TurnDegGyro;
import frc.robot.commands.UpdateAllianceColor;
import frc.robot.commands.WristSetSpeed;
import frc.robot.commands.Auto_Pattern.ChangeArmPos;
import frc.robot.commands.Auto_Pattern.ComplexAuto;
import frc.robot.commands.Auto_Pattern.Obstacle;
import frc.robot.commands.Auto_Pattern.RealComplexAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisualFeedbackSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 * //
 */
public class RobotContainer {
        // camera
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // The robot's subsystems
        private DriveSubsystem m_robotDrive = new DriveSubsystem();
        private ArmSubsystem m_robotArm = new ArmSubsystem();
        private GripperSubsystem m_robotGripper = new GripperSubsystem();
        private WristSubsystem m_wrist = new WristSubsystem();
        private VisualFeedbackSubsystem m_led = new VisualFeedbackSubsystem();

        // A chooser for autonomous commands
        SendableChooser<Command> m_chooser = new SendableChooser<>();
        SendableChooser<Command> m_driveChooser = new SendableChooser<>();

        // The driver's controller
        GenericHID m_driverJoyStick1 = new GenericHID(OIConstants.kDriverController1Port);
        GenericHID m_driverJoyStick2 = new GenericHID(OIConstants.kDriverController2Port);
        // GenericHID m_driverJoyStick = new GenericHID(1);
        GenericHID m_operatorJoyStick = new GenericHID(OIConstants.kOperatorControllerPort);

        // Joystick m_driverController = new Joystick(0);
        // POVButton m_dpadup = new POVButton(m_driverJoyStick1, 0);
        // POVButton m_dpaddown = new POVButton(m_driverJoyStick1, 180);

        private final Command m_complexAuto = new ComplexAuto(m_robotDrive);
        private final Command m_realcomplexAuto = new RealComplexAuto(m_robotDrive);

        private final Command m_Obstacle = new Obstacle(m_robotDrive);

        private final Command m_setArcade = new SetDriveMode(m_robotDrive, true);
        private final Command m_setTank = new SetDriveMode(m_robotDrive, false);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                // Set the default drive command to split-stick arcade drive

                m_wrist.setDefaultCommand(new WristSetSpeed(m_wrist, () -> m_driverJoyStick1.getRawAxis(3)));

                m_robotDrive.setDefaultCommand(
                                new DefaultDrive(
                                                m_robotDrive,
                                                () -> -m_driverJoyStick1.getRawAxis(OIConstants.kYaxis),
                                                () -> -m_driverJoyStick2.getRawAxis(OIConstants.kYaxis),
                                                () -> -m_driverJoyStick1.getRawAxis(OIConstants.kXaxis)));

                // m_robotDrive.setDefaultCommand(
                // new SmoothDrive(
                // m_robotDrive,
                // () -> -m_driverController.getLeftY(), //this changing doubles to supplier!
                // () -> -m_driverController.getLeftX()));
               
                //LOCKS ARM IN PLACE
                // m_robotArm.setDefaultCommand(
                //                 new ArmStayInPlace(m_robotArm, () -> m_robotArm.getTargetPosition()));
                
                m_robotArm.setDefaultCommand(
                                new ArmFreeAndPos(m_robotArm, 
                                () -> m_operatorJoyStick.getRawAxis(OIConstants.kYaxis),
                                () -> m_operatorJoyStick.getRawAxis(OIConstants.kSlideraxis),
                                () -> m_robotArm.getTargetPosition()));

                m_robotGripper.setDefaultCommand(
                                new AutoGrip(m_robotGripper, m_robotArm));

                m_chooser.setDefaultOption("Complex Auto", m_complexAuto);
                m_chooser.addOption("Real Complex Auto", m_realcomplexAuto);
                m_chooser.addOption("Obstacle", m_Obstacle);

                m_driveChooser.setDefaultOption("Tank", m_setTank);
                m_driveChooser.addOption("Arcade", m_setArcade);

                SmartDashboard.putData(m_chooser);
                SmartDashboard.putData(m_driveChooser);
                SmartDashboard.putData(m_robotDrive);
                SmartDashboard.putNumber("Motor voltage", IDConstants.kStatic);
                SmartDashboard.putData("UpdateAllianceColor", new UpdateAllianceColor(m_led));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                // ==========================================DRIVER
                // CONTROLS==========================================

                // DRIVE AND BALANCE
                new JoystickButton(m_driverJoyStick1, 1)
                                .toggleOnTrue(new DriveAndBalance(m_robotDrive, 36));

                // QUARTER SPEED
                new JoystickButton(m_driverJoyStick1, 2)
                                .toggleOnTrue(new QuarterDriveSpeed(m_robotDrive));

                new JoystickButton(m_driverJoyStick1, 3)
                                .onTrue(new ToggleGripper(m_robotGripper, m_robotArm));

                // =======================================================================================================

                // ==========================================OPERATOR
                // CONTROLS==========================================

                // new POVButton(m_operatorJoyStick, 0)
                // .whileTrue(new ChangeArmPos(.5, m_robotArm));

                new POVButton(m_operatorJoyStick, 180)
                                .whileTrue(new ChangeArmPos(-.5, m_robotArm));

                new JoystickButton(m_operatorJoyStick, 1)
                                .onTrue(new ArmSetPosition(m_robotArm, 50));

                new JoystickButton(m_operatorJoyStick, 1)
                                .onTrue(new ArmSetPosition(m_robotArm, -50));

                new POVButton(m_operatorJoyStick, 0)
                                .onTrue(new WristSetSpeed(m_wrist, () -> 1));

                // =======================================================================================================

                // ==========================================DEBUGING
                // CONTROLS==========================================
                // While holding the shoulder button, drive at half speed
                // kX = button b
                // kA = button x
                // kB = button a
                // kY = button y
                // new JoystickButton(m_driverController, OIConstants.buttonY.value)
                // .onTrue(new TurnDegGyro(90, .25, m_robotDrive));
                // new JoystickButton(m_driverController, OIConstants.buttonB.value)
                // .onTrue(new DriveWithinDistance(40, .35, m_robotDrive));
                // new JoystickButton(m_driverController, OIConstants.buttonB.value)
                // .onTrue(new DriveVoltage(SmartDashboard.getNumber("Motor voltage",0),
                // m_robotDrive));

                // new JoystickButton(m_driverController, OIConstants.buttonA.value)
                // .onTrue(new TurnDeg(90, .5, m_robotDrive));

                // new JoystickButton(m_driverController, Button.kRightBumper.value)
                // .toggleOnTrue(new HalveDriveSpeed(m_robotDrive));

                // new JoystickButton(m_driverController, Button.kRightBumper.value)
                // .toggleOnTrue(new PigeonBalanceSmartVelocity(m_robotDrive));

                // new JoystickButton(m_driverController, OIConstants.kButtonY)
                // .toggleOnTrue(new PIDLockInPlace(m_robotDrive, 36));

                // new JoystickButton(m_driverController, OIConstants.buttonX.value)
                // .onTrue(new DriveDistance(60, 0.25, m_robotDrive));

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
