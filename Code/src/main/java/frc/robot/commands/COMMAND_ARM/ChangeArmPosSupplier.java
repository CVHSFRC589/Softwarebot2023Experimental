// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.COMMAND_ARM;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ArmSubsystem;

// public class ChangeArmPosSupplier extends CommandBase {
//   /** Creates a new ChangeArmPos. */
//   private ArmSubsystem m_arm;
//   private double m_increment;
//   public ChangeArmPosSupplier(DoubleSupplier y, DoubleSupplier slider , ArmSubsystem arm) {
//     m_arm = arm;
//     m_increment = y.getAsDouble()*slider.getAsDouble()*.1;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_arm.incrementPosition(m_increment);
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
