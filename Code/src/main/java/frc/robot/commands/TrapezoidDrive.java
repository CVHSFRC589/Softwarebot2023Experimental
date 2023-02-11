// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapezoidDrive extends TrapezoidProfileCommand {
  /** Creates a new TrapezoidDrive. */
  public TrapezoidDrive(DriveSubsystem drive, double distance) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(2, 2),
            // Goal state
            new TrapezoidProfile.State(distance, 0),
            // Initial state
            new TrapezoidProfile.State(0,0)),
        state -> {
          drive.resetEncoders();
          // Use current trajectory state here
        });
  }
}
