// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_ARM;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmPhysicalConstants;
import frc.robot.commands.COMMAND_WRIST.WristSetSpeedTime;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristUpArmUp extends ParallelCommandGroup {
  /** Creates a new WristUpArmUp. */
  public WristUpArmUp(WristSubsystem wrist, ArmSubsystem arm, double deg) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristSetSpeedTime(wrist, () -> -0.5, 1),
      new ArmSetPosVelocity(arm, deg)
    );
  }
}
