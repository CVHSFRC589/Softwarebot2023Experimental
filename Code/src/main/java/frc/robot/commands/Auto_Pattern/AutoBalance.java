// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Pattern;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.COMMAND_DRIVE.DriveAndBalance;
import frc.robot.commands.COMMAND_DRIVE.DriveDistance;
import frc.robot.commands.COMMAND_DRIVE.PIDLockInPlace;
import frc.robot.commands.COMMAND_DRIVE.PigeonBalanceSmartVelocity;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends SequentialCommandGroup {
  /** Creates a new AutoBalance. */
  public AutoBalance(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveDistance(80, .50, drive),
      new PigeonBalanceSmartVelocity(drive),
      new PIDLockInPlace(drive, 0)
      );
  }
}
