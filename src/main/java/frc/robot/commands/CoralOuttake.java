// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.



// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.Outtake;
// import frc.robot.subsystems.Intake;

// public class CoralOuttake{
//   private Outtake outtake;
//   private Intake intake;

//   public CoralOuttake(Outtake outtake, Intake intake) {
//     this.outtake = outtake;
//     this.intake = intake;

//   }

//   public Command moveCoralToOuttake() {
//     return intake.runMotorCmd()
//         .alongWith(outtake.runMotorCmd())
//         .until(outtake::isCoral)
//         .andThen(outtake.stopMotorCmd())
//         .andThen(intake.stopMotorCmd());
//   }
// }
