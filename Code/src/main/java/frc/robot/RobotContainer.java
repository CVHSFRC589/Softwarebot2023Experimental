// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto_Pattern.*;
import frc.robot.commands.COMMAND_ARM.*;
import frc.robot.commands.COMMAND_DRIVE.*;
import frc.robot.commands.COMMAND_GRIP.*;
import frc.robot.commands.COMMAND_MISC.*;
import frc.robot.commands.COMMAND_WRIST.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 * 
 */

// ==========================================TODO
// LIST==========================================
/*
 * add logic to arm clamps
 * same logic to wrist and mindful of arm position(are there dangerous wrist
 * positions???)
 * 
 * 
 * 
 */
// ====================================================================================

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
        SendableChooser<Command> m_driveChooser = new SendableChooser<>();
        SendableChooser<Command> m_autoChooser = new SendableChooser<>();


        // The driver's joysticks
        GenericHID m_driverJoyStickLeft = new GenericHID(OIConstants.kDriverController1Port);
        GenericHID m_driverJoyStickRight = new GenericHID(OIConstants.kDriverController2Port);

        GenericHID m_operatorJoyStick = new GenericHID(OIConstants.kOperatorControllerPort);

        private final Command m_defaultAuto = new AutoBalance(m_robotDrive);

        // private final Command m_setArcade = new SetDriveMode(m_robotDrive, true);
        // private final Command m_setTank = new SetDriveMode(m_robotDrive, false);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                // // Set the default drive command to split-stick arcade drive
                m_wrist.setDefaultCommand(new WristStayInPlace(m_wrist, () -> m_wrist.getWristDeg()));

                m_robotDrive.setDefaultCommand(
                                new DriveDefault(
                                                m_robotDrive,
                                                () -> -m_driverJoyStickLeft.getRawAxis(OIConstants.kYaxis),
                                                () -> -m_driverJoyStickRight.getRawAxis(OIConstants.kYaxis),
                                                () -> -m_driverJoyStickLeft.getRawAxis(OIConstants.kXaxis)));


                // LOCKS ARM IN PLACE
                m_robotArm.setDefaultCommand(
                                new ArmFollowJoy(m_robotArm,
                                                () -> -m_operatorJoyStick.getRawAxis(OIConstants.kYaxis),
                                                () -> -m_operatorJoyStick.getRawAxis(OIConstants.kSlideraxis)));

                // m_driveChooser.setDefaultOption("Tank", m_setTank);
                // m_driveChooser.addOption("Arcade", m_setArcade);
                m_autoChooser.setDefaultOption("Drive out and Balance", m_defaultAuto);
                m_autoChooser.addOption("Leave Community", new LeaveCommunity(m_robotDrive));
                m_autoChooser.addOption("Score level 3 Cube", new Score3Cube(m_robotArm, m_robotDrive, m_robotGripper));

                SmartDashboard.putData(m_autoChooser);
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

                // ==========================================DRIVER CONTROLS========================================== \\

                // DRIVE AND BALANCE
                new JoystickButton(m_driverJoyStickLeft, 11)
                                .toggleOnTrue(new DriveAndBalance(m_robotDrive, 36));
                // LOCK
                new JoystickButton(m_driverJoyStickLeft, 10)
                                .toggleOnTrue(new PIDLockInPlace(m_robotDrive, 0));

                // 75% SPEED
                new JoystickButton(m_driverJoyStickLeft, 6)
                                .toggleOnTrue(new DriveSelectSpeed(m_robotDrive, .75));
                // 50% SPEED
                new JoystickButton(m_driverJoyStickLeft, 7)
                                .toggleOnTrue(new DriveSelectSpeed(m_robotDrive, .50));

                // TOGGLE GRIPPER
                new JoystickButton(m_driverJoyStickRight, 1)
                                .onTrue(new ToggleGripper(m_robotGripper));

                // ===================================================================================================== \\

                // ==========================================OPERATOR CONTROLS========================================== \\

                // ZERO ARM ENCODER
                new JoystickButton(m_operatorJoyStick, 12)
                                .toggleOnTrue(new ArmZeroEncoder(m_robotArm));
                // LOCK ARM IN PLACE
                new JoystickButton(m_operatorJoyStick, 2)
                                .toggleOnTrue(new ArmPIDLock(m_robotArm));
                // LEVEL MIN
                new JoystickButton(m_operatorJoyStick, 1)
                                .toggleOnTrue(new ArmSetPosVelocity(m_robotArm, 0));
                // LEVEL MAX
                new JoystickButton(m_operatorJoyStick, 3)
                                .toggleOnTrue(new ArmSetPosVelocity(m_robotArm, ArmPhysicalConstants.level2));
                // LEVEL FLOOR
                new JoystickButton(m_operatorJoyStick, 4)
                                .toggleOnTrue(new ArmSetPosVelocity(m_robotArm, ArmPhysicalConstants.level1));
                // LEVEL LOAD
                new JoystickButton(m_operatorJoyStick, 5)
                                .toggleOnTrue(new ArmSetPosVelocity(m_robotArm, ArmPhysicalConstants.levelportal));

                // TOGGLE PISTON
                // new JoystickButton(m_operatorJoyStick, 8)
                //                 .toggleOnTrue(new ArmTogglePiston(m_robotArm));
                // WRIST UP
                new JoystickButton(m_operatorJoyStick, 9)
                                .whileTrue(new WristSetSpeed(m_wrist, () -> -0.5));
                // WRIST DOWN
                new JoystickButton(m_operatorJoyStick, 10)
                                .whileTrue(new WristSetSpeed(m_wrist, () -> 0.5));
                // ======================================================================================================= \\

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return m_autoChooser.getSelected();
        }
}
