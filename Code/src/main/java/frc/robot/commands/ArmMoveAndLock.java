// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.Constants.OIConstants;
// import frc.robot.commands.Auto_Pattern.ChangeArmPos;
// import frc.robot.subsystems.ArmSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ArmMoveAndLock extends ParallelCommandGroup {

//   /** Creates a new ArmMoveAndLock. */
//   public ArmMoveAndLock(ArmSubsystem arm, GenericHID opjoy){//, DoubleSupplier y, DoubleSupplier slider) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
    
//     addCommands(
//         new ChangeArmPosSupplier(() -> opjoy.getRawAxis(OIConstants.kYaxis),
//                                   () -> opjoy.getRawAxis(OIConstants.kSlideraxis),
//         arm),
//         new ArmStayInPlace(arm, () -> arm.getTargetPosition()));

//   }
// }
