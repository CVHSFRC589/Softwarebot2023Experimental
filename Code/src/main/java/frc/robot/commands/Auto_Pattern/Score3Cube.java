// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Pattern;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.commands.COMMAND_ARM.ArmSetPosVelocity;
import frc.robot.commands.COMMAND_ARM.ArmZeroEncoder;
import frc.robot.commands.COMMAND_DRIVE.DriveDistance;
import frc.robot.commands.COMMAND_GRIP.CloseGripper;
import frc.robot.commands.COMMAND_GRIP.OpenGripper;
import frc.robot.commands.COMMAND_MISC.Pause;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score3Cube extends SequentialCommandGroup {
  /** Creates a new Score3Cube. */
  public Score3Cube(ArmSubsystem arm, DriveSubsystem drive, GripperSubsystem grip) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new CloseGripper(grip),
      new DriveDistance(36, .5, drive),
      new Pause(1),
      new ArmZeroEncoder(arm),
      new Pause(.3),
      new ArmSetPosVelocity(arm, ArmPhysicalConstants.level2),
      new OpenGripper(grip),
      new ArmSetPosVelocity(arm, 0)
    );
  }
}
