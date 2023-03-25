// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Pattern;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.commands.COMMAND_ARM.ArmExtendPiston;
import frc.robot.commands.COMMAND_ARM.ArmInWristIn;
import frc.robot.commands.COMMAND_ARM.ArmRetractPiston;
import frc.robot.commands.COMMAND_ARM.ArmSetPosVelocity;
import frc.robot.commands.COMMAND_ARM.ArmZeroEncoder;
import frc.robot.commands.COMMAND_ARM.WristUpArmUp;
import frc.robot.commands.COMMAND_DRIVE.DriveDistance;
import frc.robot.commands.COMMAND_DRIVE.PIDLockInPlace;
import frc.robot.commands.COMMAND_DRIVE.PigeonBalanceSmartVelocity;
import frc.robot.commands.COMMAND_GRIP.CloseGripper;
import frc.robot.commands.COMMAND_GRIP.OpenGripper;
import frc.robot.commands.COMMAND_MISC.Pause;
import frc.robot.commands.COMMAND_WRIST.WristSetSpeedTime;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLevel1AndBalanceBackwards extends SequentialCommandGroup {
  /** Creates a new Score3Cube. */
  public ScoreLevel1AndBalanceBackwards(ArmSubsystem arm, DriveSubsystem drive, GripperSubsystem grip, WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      //SCORE THREE CUBE
      new ArmZeroEncoder(arm),
      new CloseGripper(grip),
      new ArmRetractPiston(arm),
      // new WristSetSpeedTime(wrist, () -> -0.5, 1),
      // new ArmSetPosVelocity(arm, ArmPhysicalConstants.level2),
      new WristUpArmUp(wrist, arm, ArmPhysicalConstants.level1),
      new WristSetSpeedTime(wrist, () -> 0.5, 1.5),
      new WaitCommand(0.25),
      new OpenGripper(grip),
      new WaitCommand(0.5),
      // new WaitCommand(.1),
      new DriveDistance(14, -0.5, drive),
      new CloseGripper(grip),


      // new WristSetSpeedTime(wrist, () -> -0.5, 1),
      // new ArmSetPosVelocity(arm, 0),
      new ArmInWristIn(wrist, arm),
      // new WaitCommand(.25),
      new ArmExtendPiston(arm),
      //BALANCE BACKWARDS

      new DriveDistance(90, -0.65, drive),
      
      new DriveDistance(33, -0.5, drive),
      // new DriveDistance(10, -0.3, drive),
      new WaitCommand(.15),
      new DriveDistance(70, 0.6, drive),
      new PigeonBalanceSmartVelocity(drive),
      new PIDLockInPlace(drive, 0)
    );
  }
}
