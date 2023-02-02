// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Pattern;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.TurnDeg;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Obstacle extends SequentialCommandGroup {
  /** Creates a new Obstacle. */
  public Obstacle(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveDistance(24, .35, drive),

        new TurnDeg(90, -0.35, drive),

        new DriveDistance(32, .35, drive),
        
        new TurnDeg(90, 0.35, drive),

        new DriveDistance(57, .35, drive),
        
        new TurnDeg(90, 0.35, drive),

        new DriveDistance(58, .35, drive),

        new TurnDeg(90, 0.35, drive),
       
        new DriveDistance(55, .35, drive),

        new TurnDeg(80, 0.35, drive),

        new DriveDistance(30, .35, drive),

        new TurnDeg(90, -0.35, drive),

        new DriveDistance(20, .35, drive)


    );
  }
}
