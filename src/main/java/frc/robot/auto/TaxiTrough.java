// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.auto;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Outtake;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TaxiTrough extends SequentialCommandGroup {
//   /** Creates a new TaxiTrough. */
//   public TaxiTrough(Drive drivetrain, Elevator elevator, Intake intake, Outtake outtake) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//         new InstantCommand(() -> drivetrain.zeroHeading()),
//         new RunCommand(() -> drivetrain.drive(() -> -0.2, () -> 0.0, () -> 0.0), drivetrain)
//           .withTimeout(2),
        

//     );
//   }
// }
