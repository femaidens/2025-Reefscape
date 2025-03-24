// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class CoralTransition{
  private Intake intake;
  private Outtake outtake;

  public CoralTransition(Intake intake, Outtake outtake) {
    this.intake = intake;
    this.outtake = outtake;

  }

  public Command moveCoralToOuttake() {
  //   return intake.runMotorCmd()
  //       .alongWith(outtake.runMotorCmd())
  //       .until(outtake::isCoral)
  //       .andThen(outtake.stopMotorCmd())
  //       .andThen(intake.stopMotorCmd());
        
  // }
  return outtake.runMotorCmd()
        .until(outtake::isCoral)
        .andThen(outtake.stopMotorCmd());
  }
}
