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
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.OIConstants;
// import frc.robot.commands.*;
import frc.robot.commands.Auto_Pattern.*;
import frc.robot.commands.COMMAND_ARM.*;
// import frc.robot.commands.COMMAND_ARM.ArmFollowJoy;
// import frc.robot.commands.COMMAND_ARM.ArmSetPosition;
import frc.robot.commands.COMMAND_DRIVE.*;
import frc.robot.commands.COMMAND_GRIP.CloseGripper;
import frc.robot.commands.COMMAND_GRIP.OpenGripper;
// import frc.robot.commands.COMMAND_DRIVE.DriveDefault;
// import frc.robot.commands.COMMAND_DRIVE.DriveAndBalance;
// import frc.robot.commands.COMMAND_DRIVE.DriveSelectSpeed;
// import frc.robot.commands.COMMAND_DRIVE.SetDriveMode;
import frc.robot.commands.COMMAND_GRIP.ToggleGripper;
import frc.robot.commands.COMMAND_MISC.UpdateAllianceColor;
import frc.robot.commands.COMMAND_WRIST.WristSetSpeed;
import frc.robot.commands.COMMAND_WRIST.WristStayInPlace;
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


 // ==========================================TODO LIST==========================================
        /* add logic to arm clamps
         * same logic to wrist and mindful of arm position(are there dangerous wrist positions???)
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

                m_wrist.setDefaultCommand(new WristStayInPlace(m_wrist, () -> m_wrist.getWristDeg()));

                m_robotDrive.setDefaultCommand(
                                new DriveDefault(
                                                m_robotDrive,
                                                () -> -m_driverJoyStick1.getRawAxis(OIConstants.kYaxis),
                                                () -> -m_driverJoyStick2.getRawAxis(OIConstants.kYaxis),
                                                () -> -m_driverJoyStick1.getRawAxis(OIConstants.kXaxis)));

                // m_robotDrive.setDefaultCommand(
                // new SmoothDrive(
                // m_robotDrive,
                // () -> -m_driverController.getLeftY(), //this changing doubles to supplier!
                // () -> -m_driverController.getLeftX()));
               
                // LOCKS ARM IN PLACE
                m_robotArm.setDefaultCommand(
                                new ArmFollowJoy(m_robotArm, 
                                                () -> -m_operatorJoyStick.getRawAxis(OIConstants.kYaxis),
                                                () -> -m_operatorJoyStick.getRawAxis(OIConstants.kSlideraxis)));
                
                // m_robotArm.setDefaultCommand(
                //                 new ArmMoveAndLock(m_robotArm,m_operatorJoyStick
                //                                 // () -> -m_operatorJoyStick.getRawAxis(OIConstants.kYaxis),
                //                                 // () -> m_operatorJoyStick.getRawAxis(OIConstants.kSlideraxis)
                //                                 ));

                // m_robotGripper.setDefaultCommand(
                //                 new AutoGrip(m_robotGripper, m_robotArm));

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

                // ==========================================DRIVER CONTROLS==========================================

                // DRIVE AND BALANCE
                new JoystickButton(m_driverJoyStick1, 11)
                                .toggleOnTrue(new DriveAndBalance(m_robotDrive, 36));

                // QUARTER SPEED
                new JoystickButton(m_driverJoyStick1, 10)
                                .toggleOnTrue(new DriveSelectSpeed(m_robotDrive, .75));
                //TOGGLE GRIPPER
                new JoystickButton(m_driverJoyStick1, 3)
                                .onTrue(new OpenGripper(m_robotGripper, m_robotArm));

                new JoystickButton(m_driverJoyStick1, 4)
                                .onTrue(new CloseGripper(m_robotGripper, m_robotArm));


                // =====================================================================================================

                // ==========================================OPERATOR CONTROLS==========================================
                
                // new POVButton(m_operatorJoyStick, 0)
                // .whileTrue(new ChangeArmPos(.5, m_robotArm));

                // new POVButton(m_operatorJoyStick, 180)
                //                 .whileTrue(new ChangeArmPos(-.5, m_robotArm));


                //MAX ARM VALUE GO TO MAX HEIGHT
                new JoystickButton(m_operatorJoyStick, 1)
                                .toggleOnTrue(new ArmSetPosition(m_robotArm, 50));
                //MIN ARM VALUE GO TO BASE/INSIDE
                new JoystickButton(m_operatorJoyStick, 2)
                                .toggleOnTrue(new ArmSetPosition(m_robotArm, -50));
                //CANCEL FIXED POSITION - RETURN TO JOYSTICK FOLLOW
                new JoystickButton(m_operatorJoyStick, 4)
                                .toggleOnTrue(new ArmCancelFixedPosition(m_robotArm));
                //LOCK ARM IN PLACE
                new JoystickButton(m_operatorJoyStick, 3)
                                .toggleOnTrue(new ArmSetPosition(m_robotArm, m_robotArm.getTargetPosition()));
                
                new JoystickButton(m_operatorJoyStick, 12)
                                .toggleOnTrue(new ArmZeroEncoder(m_robotArm));
                //wrist 
                new POVButton(m_operatorJoyStick, 0)
                                .whileTrue(new WristSetSpeed(m_wrist, () -> -.5));
                                new POVButton(m_operatorJoyStick, 180)
                                .whileTrue(new WristSetSpeed(m_wrist, () -> 0.5));

                                        //==============ARM + WRIST PRESETS==============\\
                                        new JoystickButton(m_operatorJoyStick, 5)
                                        .toggleOnTrue(new ArmSetPosition(m_robotArm, 5));
                                // new JoystickButton(m_operatorJoyStick, 6)
                                //         .toggleOnTrue(new ArmSetPosition(m_robotArm, ArmPhysicalConstants.level2cone));
                                // new JoystickButton(m_operatorJoyStick, 7)
                                //         .toggleOnTrue(new ArmSetPosition(m_robotArm, ArmPhysicalConstants.level2cube));
                                new JoystickButton(m_operatorJoyStick, 6)
                                        .toggleOnTrue(new ArmSetPosition(m_robotArm, 0.05));
                                new JoystickButton(m_operatorJoyStick, 7)
                                        .toggleOnTrue(new ArmSetPosition(m_robotArm, 0.15));
                                new JoystickButton(m_operatorJoyStick, 8)
                                        .toggleOnTrue(new ArmSetPosition(m_robotArm, ArmPhysicalConstants.levelportal));
                                new JoystickButton(m_operatorJoyStick, 9)
                                        .toggleOnTrue(new ArmSetPosition(m_robotArm, ArmPhysicalConstants.levelfloor));
                                new JoystickButton(m_operatorJoyStick, 10)
                                        .toggleOnTrue(new ArmSetPosition(m_robotArm, ArmPhysicalConstants.level1conecube));
                // =======================================================================================================

                // ==========================================DEBUGING CONTROLS==========================================
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

                new JoystickButton(m_driverJoyStick1, 11)
                .toggleOnTrue(new DriveAndBalance(m_robotDrive));

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
